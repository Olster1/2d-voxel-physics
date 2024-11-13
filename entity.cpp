enum EntityType {
    ENTITY_NONE,
    ENTITY_PLAYER,
    ENTITY_PICKUP_ITEM,
    ENTITY_GRASS_SHORT,
    ENTITY_GRASS_LONG,
    ENTITY_FOX,
    ENTITY_HORSE,
};

enum BlockType {
    BLOCK_NONE,
    BLOCK_GRASS,
    BLOCK_SOIL,
    BLOCK_STONE,
    BLOCK_CLOUD,
    BLOCK_TREE_WOOD,
    BLOCK_TREE_LEAVES,
    BLOCK_WATER,
    BLOCK_GRASS_SHORT_ENTITY,
    BLOCK_GRASS_TALL_ENTITY,
    BLOCK_COAL,
    BLOCK_IRON,
    BLOCK_OUTLINE,


    //NOTHING PAST HERE
    BLOCK_TYPE_COUNT
};

struct TimeOfDayValues {
    float4 skyColorA;
    float4 skyColorB;
};

enum VoxelPhysicsFlag {
    VOXEL_NONE = 0,
    VOXEL_OCCUPIED = 1 << 0,
    VOXEL_CORNER = 1 << 1,
    VOXEL_EDGE = 1 << 2,
    VOXEL_INSIDE = 1 << 3,
    VOXEL_COLLIDING = 1 << 4,
};

struct EntityID {
    size_t stringSizeInBytes; //NOTE: Not include null terminator
    char stringID[256]; 
    uint32_t crc32Hash;
};

static int global_entityIdCreated = 0;

EntityID makeEntityId(int randomStartUpID) {
    EntityID result = {};

    time_t timeSinceEpoch = time(0);

    #define ENTITY_ID_PRINTF_STRING "%ld-%d-%d", timeSinceEpoch, randomStartUpID, global_entityIdCreated

    //NOTE: Allocate the string
    size_t bufsz = snprintf(NULL, 0, ENTITY_ID_PRINTF_STRING) + 1;
    assert(bufsz < arrayCount(result.stringID));
    result.stringSizeInBytes = MathMin_sizet(arrayCount(result.stringID), bufsz);
    snprintf(result.stringID, bufsz, ENTITY_ID_PRINTF_STRING);

    result.crc32Hash = get_crc32(result.stringID, result.stringSizeInBytes);

    #undef ENTITY_ID_PRINTF_STRING

    //NOTE: This would have to be locked in threaded application
    global_entityIdCreated++;

    return result;
} 

bool areEntityIdsEqual(EntityID a, EntityID b) {
    bool result = false;
    if(a.crc32Hash == b.crc32Hash && easyString_stringsMatch_nullTerminated(a.stringID, b.stringID)) {
        result = true;
    }
    return result;
}

struct VoxelEntity {
    EntityID id;
    TransformX T;

    //NOTE: Physics details
    float3 dP;
    float3 ddPForFrame;

    float inverseMass;
    float inverseMomentOfInteria;
    float coefficientOfRestitution;
    float friction;

    //NOTE: Voxel data
    float2 worldBounds;

    float sleepTimer;
    bool asleep;

    u8 *data;
    int stride;
    int pitch;
};

#include "./physics.cpp"

u8 getByteFromVoxelEntity(VoxelEntity *e, int x, int y) {
    return e->data[y*e->stride + x];
}

bool isVoxelOccupied(VoxelEntity *e, int x, int y) {
    bool result = false;
    if(x >= 0 && x < e->stride && y >= 0 && y < e->pitch) {
        result = (e->data[y*e->stride + x] & VOXEL_OCCUPIED);
    }
    return result;
}


float2 getVoxelPositionInModelSpace(float2 pos) {
    pos.x *= VOXEL_SIZE_IN_METERS;
    pos.y *= VOXEL_SIZE_IN_METERS;

    //NOTE: To account for a voxel posiiton being in the center of a voxel
    pos.x += 0.5f*VOXEL_SIZE_IN_METERS;
    pos.y += 0.5f*VOXEL_SIZE_IN_METERS;
    return pos;
}

float3 voxelToWorldP(VoxelEntity *e, int x, int y) {
    float3 center = make_float3(0.5f*e->worldBounds.x, 0.5f*e->worldBounds.y, 0);
    const float3 worldP = e->T.pos;
    float2 modelSpace = getVoxelPositionInModelSpace(make_float2(x, y));
    float3 diffFromCenter = minus_float3(make_float3(modelSpace.x, modelSpace.y, 0), center);
    float3 p = plus_float3(worldP, diffFromCenter);

    return p;
}

CollisionPoint doesVoxelCollide(PhysicsWorld *physicsWorld, float2 worldP, VoxelEntity *e, int idX, int idY, bool swap) {
    CollisionPoint result;
    result.x = -1;
    result.y = -1;

    float2 localP = minus_float2(worldP, e->T.pos.xy);
    float2 center = make_float2(0.5f*e->worldBounds.x, 0.5f*e->worldBounds.y);

    float2 p = plus_float2(localP, center);

    //NOTE: convert to voxel space
    p.x *= VOXELS_PER_METER;
    p.y *= VOXELS_PER_METER;

    int x = (int)(p.x);
    int y = (int)(p.y);

    float2 voxels[9] = {
        make_float2(0, 0), 
        make_float2(1, 0), 
        make_float2(-1, 0), 
        make_float2(0, 1), 
        make_float2(0, -1), 
        make_float2(1, 1), 
        make_float2(-1, -1), 
        make_float2(-1, 1), 
        make_float2(1, -1), 
    };  

    float smallestDistance = FLT_MAX;

    for(int i = 0; i < arrayCount(voxels); ++i) {
        float2 voxelSpace = plus_float2(make_float2(x, y), voxels[i]);

        int testX = (int)voxelSpace.x;
        int testY = (int)voxelSpace.y;

        float2 voxelWorldP = voxelToWorldP(e, testX, testY).xy;

        if(isVoxelOccupied(e, testX, testY)) {
            float2 diff = minus_float2(worldP, voxelWorldP);

            if(swap) {
                diff = minus_float2(voxelWorldP, worldP);
            }

            float distanceSqr = float2_dot(diff, diff);

            if(distanceSqr < smallestDistance && distanceSqr <= VOXEL_SIZE_IN_METERS_SQR) {
                smallestDistance = distanceSqr;

                result.entityId = e->id;
                result.x = idX;
                result.y = idY;
                result.point = lerp_float2(worldP, voxelWorldP, 0.5f);
                result.normal = normalize_float2(diff);
                result.seperation = sqrt(distanceSqr) - VOXEL_SIZE_IN_METERS;
                result.Pn = 0;
                result.inverseMassNormal = 0;
                result.velocityBias = 0;
            }
        }
    }

    return result;
}

void collideVoxelEntities(PhysicsWorld *physicsWorld, VoxelEntity *a, VoxelEntity *b) {
    int pointCount = 0;
    CollisionPoint points[MAX_CONTACT_POINTS_PER_PAIR];

    //NOTE: Keep the order consistent with the order in the arbiter
    if(a > b) {
        VoxelEntity *temp = b;
        b = a;
        a = temp;
    }
    
    //NOTE: Check corners with corners & edges first
    for(int y = 0; y < a->pitch; y++) {
        for(int x = 0; x < a->stride; x++) {
            u8 byte = getByteFromVoxelEntity(a, x, y);
            a->data[y*a->stride + x] &= ~(VOXEL_COLLIDING);
            if(byte & VOXEL_CORNER) {
                CollisionPoint p = doesVoxelCollide(physicsWorld, voxelToWorldP(a, x, y).xy, b, x, y, true);

                if(p.x >= 0) {
                    //NOTE: Wake up the entity 
                    // wakeUpEntity(a);
                    // wakeUpEntity(b);
                    
                    //NOTE: Found a point
                    a->data[y*a->stride + x] |= VOXEL_COLLIDING;
                    assert(pointCount < arrayCount(points));
                    if(pointCount < arrayCount(points)) {
                        points[pointCount++] = p;
                    }
                }
            }
        }
    }

    //NOTE: Check corners with corners & edges first
    for(int y = 0; y < b->pitch; y++) {
        for(int x = 0; x < b->stride; x++) {
            b->data[y*b->stride + x] &= ~(VOXEL_COLLIDING);
            u8 byte = getByteFromVoxelEntity(b, x, y);
            if(byte & VOXEL_CORNER) {
                CollisionPoint p = doesVoxelCollide(physicsWorld, voxelToWorldP(b, x, y).xy, a, x, y, false);

                if(p.x >= 0) {
                    b->data[y*b->stride + x] |= VOXEL_COLLIDING;
                    //NOTE: Wake up the entity 
                    // wakeUpEntity(a);
                    // wakeUpEntity(b);
                    
                    //NOTE: Found a point
                    assert(pointCount < arrayCount(points));
                    if(pointCount < arrayCount(points)) {
                        points[pointCount++] = p;

                    }
                }
            }
        }
    }

    mergePointsToArbiter(physicsWorld, points, pointCount, a, b);
}


void classifyPhysicsShape(VoxelEntity *e) {
    for(int y = 0; y < e->pitch; y++) {
        for(int x = 0; x < e->stride; x++) {
            u8 flags = e->data[y*e->stride + x];

            if(flags & VOXEL_OCCUPIED) {
                //NOTE: Clear flags
                flags &= ~(VOXEL_CORNER | VOXEL_EDGE | VOXEL_INSIDE);

                bool found = false;
                //NOTE: Check whether corner
                float2 corners[4] = {make_float2(1, 1), make_float2(-1, 1), make_float2(1, -1), make_float2(-1, -1)};
                for(int i = 0; i < arrayCount(corners) && !found; ++i) {
                    float2 corner = corners[i];
                    if(!isVoxelOccupied(e, x + corner.x, y + corner.y) && !isVoxelOccupied(e, x + corner.x, y) && !isVoxelOccupied(e, x, y + corner.y)) {
                        flags |= VOXEL_CORNER;
                        found = true;
                    }
                }   
                
                if(!found) {
                    //NOTE: Check whether edge
                    if(!isVoxelOccupied(e, x + 1, y) || !isVoxelOccupied(e, x - 1 , y) || !isVoxelOccupied(e, x, y + 1) || !isVoxelOccupied(e, x, y - 1)) {
                        flags |= VOXEL_EDGE;
                        found = true;
                    }
                }

                if(!found) {
                    flags |= VOXEL_INSIDE;
                }

                e->data[y*e->stride + x] = flags;
            }
        }
    }
}

VoxelEntity createVoxelCircleEntity(float radius, float3 pos, float inverseMass, int randomStartUpID) {
    VoxelEntity result = {};

    result.id = makeEntityId(randomStartUpID);

    result.sleepTimer = 0;
    result.asleep = false;
    
    result.friction = 0.2f;
    result.T.pos = pos;
    result.inverseMass = inverseMass;
    result.coefficientOfRestitution = 0.8f;

    float diameter = 2*radius;
    result.worldBounds = make_float2(diameter, diameter);
    
    int diameterInVoxels = round(diameter*VOXELS_PER_METER);
    int t = (int)(diameterInVoxels*diameterInVoxels);
    result.data = (u8 *)easyPlatform_allocateMemory(sizeof(u8)*t, EASY_PLATFORM_MEMORY_ZERO);

    result.stride = diameterInVoxels;
    result.pitch = diameterInVoxels;

    float2 center = make_float2(radius, radius);
    for(int y = 0; y < result.pitch; y++) {
        for(int x = 0; x < result.stride; x++) {
            float2 pos = getVoxelPositionInModelSpace(make_float2(x, y));

            float2 diff = minus_float2(pos, center);

            u8 flags = VOXEL_NONE;

            if(float2_magnitude(diff) <= radius) {
                flags |= VOXEL_OCCUPIED;
            } 

            result.data[y*result.stride + x] = flags;
        }
    }

    classifyPhysicsShape(&result);

    return result;
}

VoxelEntity createVoxelPlaneEntity(float length, float3 pos, float inverseMass, int randomStartUpID) {
    VoxelEntity result = {};

    result.id = makeEntityId(randomStartUpID);

    result.T.pos = pos;
    result.inverseMass = inverseMass;
    result.coefficientOfRestitution = 0;
    result.friction = 0.2f;

    result.sleepTimer = 0;
    result.asleep = false;

    result.worldBounds = make_float2(length, 30*VOXEL_SIZE_IN_METERS);

    result.stride = result.worldBounds.x*VOXELS_PER_METER;
    result.pitch = result.worldBounds.y*VOXELS_PER_METER;

    int areaInVoxels = result.stride*result.pitch;
    
    result.data = (u8 *)easyPlatform_allocateMemory(sizeof(u8)*areaInVoxels, EASY_PLATFORM_MEMORY_ZERO);

    for(int y = 0; y < result.pitch; y++) {
        for(int x = 0; x < result.stride; x++) {
            result.data[y*result.stride + x] = VOXEL_OCCUPIED;
        }
    }

    classifyPhysicsShape(&result);

    return result;
}

VoxelEntity createVoxelSquareEntity(float w, float h, float3 pos, float inverseMass, int randomStartUpID) {
    VoxelEntity result = {};

    result.id = makeEntityId(randomStartUpID);

    result.T.pos = pos;
    result.inverseMass = inverseMass;
    result.coefficientOfRestitution = 0.9f;
    result.friction = 0.2f;

    result.sleepTimer = 0;
    result.asleep = false;

    result.worldBounds = make_float2(w, h);

    result.stride = result.worldBounds.x*VOXELS_PER_METER;
    result.pitch = result.worldBounds.y*VOXELS_PER_METER;

    int areaInVoxels = result.stride*result.pitch;
    
    result.data = (u8 *)easyPlatform_allocateMemory(sizeof(u8)*areaInVoxels, EASY_PLATFORM_MEMORY_ZERO);

    for(int y = 0; y < result.pitch; y++) {
        for(int x = 0; x < result.stride; x++) {
            result.data[y*result.stride + x] = VOXEL_OCCUPIED;
        }
    }

    classifyPhysicsShape(&result);

    return result;
}

struct Block {
    u8 type;

    //NOTE: Local to the Chunk they're in
    int x;
    int y;
    int z;
    volatile uint64_t aoMask; //NOTE: Multiple threads can change this
    //NOTE: Instead of storing this 
    bool exists;

    float timeLeft;
};

struct CloudBlock {
    //NOTE: Local to the Chunk they're in
    int x;
    int z;
};

enum EntityFlags {
    SHOULD_ROTATE = 1 << 0,
    ENTITY_DESTRUCTIBLE = 1 << 1,
    ENTITY_DELETED = 1 << 2,
};

struct Entity {
    EntityID id;

    float3 offset;
    float floatTime;

    float stamina;
    
    TransformX T;
    float3 dP;
    float3 recoverDP; //NOTE: This is a seperate dP that gets applied no matter what i.e. it doesn't get stopped by collisions. It's so if an entity gets stuck in a block it can move out of it.
    EntityType type;

    Rect3f collisionBox;
    bool grounded;
    bool tryJump;
    bool running;

    AnimationState animationState;

    uint32_t flags;
    BlockType itemType;
};

#define CHUNK_DIM 16
#define BLOCKS_PER_CHUNK CHUNK_DIM*CHUNK_DIM*CHUNK_DIM

struct CloudChunk {
    int x;
    int z;

    int cloudCount;
    CloudBlock clouds[CHUNK_DIM*CHUNK_DIM];

    CloudChunk *next;
};


enum ChunkGenerationState {
    CHUNK_NOT_GENERATED = 1 << 0, 
    CHUNK_GENERATING = 1 << 1, 
    CHUNK_GENERATED = 1 << 2, 
    CHUNK_MESH_DIRTY = 1 << 3, 
    CHUNK_MESH_BUILDING = 1 << 4, 
};

struct Chunk {
    int x;
    int y;
    int z;

    volatile int64_t generateState; //NOTE: Chunk might not be generated, so check first when you get one

    //NOTE: 16 x 16 x 16
    //NOTE: Z Y X
    Block *blocks; //NOTE: Could be null

    Entity *entities;

    ChunkModelBuffer modelBuffer;
    ChunkModelBuffer alphaModelBuffer;

    Chunk *next;
};

struct ChunkInfo {
    Chunk *chunk;

    ChunkInfo *next;
};

struct BlockChunkPartner {
    Block *block;
    Chunk *chunk;

    int blockIndex;

    float3 sideNormal;
};

struct Camera {
    TransformX T;
    float fov;
    float targetFov;
    float shakeTimer;
    float runShakeTimer;
    bool followingPlayer;
};

void initBaseEntity(Entity *e, int randomStartUpID) {
    e->id = makeEntityId(randomStartUpID);
    e->T.rotation = make_float3(0, 0, 0);
    e->recoverDP = e->dP = make_float3(0, 0, 0);
    
}

Entity *initPlayer(Entity *e, int randomStartUpID) {
    initBaseEntity(e, randomStartUpID);
    e->T.pos = make_float3(0, 0, 0);
    float playerWidth = 0.7f;
    e->T.scale = make_float3(playerWidth, 1.7f, playerWidth);
    // e->T.scale = make_float3(1, 1, 1);
    
    e->type = ENTITY_PLAYER;
    e->offset = make_float3(0, 0, 0);
    e->grounded = false;
    e->flags = 0;
    e->stamina = 1;
    return e;
}