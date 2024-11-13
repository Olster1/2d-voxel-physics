#include "./game_defines.h"

uint64_t getBitWiseFlag(uint64_t bitWise) {
    return ((uint64_t)1 << bitWise);
}

static float global_fogFarDistance;
static float global_fogSeeDistance;

#include "./easy_memory.h"
#include "./resize_array.cpp"
#include "./memory_arena.h"
#include "./easy_string_utf8.h"
#include "./easy_string.h"
#include "./easy_files.h"
#include "./easy_lex.h"
#include "./imgui.h"
#include "./render.h"
#include "./transform.cpp"
#include "./animation.h"
#include "./entity.cpp"
#include "./render.cpp"
#include "./opengl.cpp"
#include "./font.cpp"
#include "./particles.cpp"
#include "./load_gltf.cpp"


Renderer *initRenderer(Texture grassTexture, Texture breakBlockTexture, Texture atlasTexture) {
    Renderer *renderer = (Renderer *)malloc(sizeof(Renderer));
    
    renderer->cubeCount = 0;
    renderer->atlasQuadCount = 0;
    renderer->glyphCount = 0;
    renderer->terrainTextureHandle = grassTexture.handle;
    renderer->breakBlockTexture = breakBlockTexture.handle;
    renderer->atlasTexture = atlasTexture.handle;

    renderer->blockShader = loadShader(blockVertexShader, blockFragShader);
    renderer->blockGreedyShader = loadShader(blockGreedyVertexShader, blockFragShader);
    renderer->quadTextureShader = loadShader(quadVertexShader, quadTextureFragShader);
    renderer->fontTextureShader = loadShader(quadVertexShader, fontTextureFragShader);

    renderer->rayCastShader = loadShader(fullScreenVertexShader, rayCastFragShader);
    renderer->skyboxShader = loadShader(skyboxVertexShader, skyboxFragShader);
    renderer->quadShader = loadShader(quadVertexShader, quadFragShader);
    renderer->blockPickupShader = loadShader(blockPickupVertexShader, blockPickupFragShader);
    renderer->skeletalModelShader = loadShader(skeletalVertexShader, skeletalFragShader);
    renderer->blockSameTextureShader = loadShader(blockSameTextureVertexShader, blockPickupFragShader);
    renderer->blockColorShader = loadShader(blockVertexShader, blockFragShader);
    
    renderer->blockModel = generateVertexBuffer(global_cubeData, 24, global_cubeIndices, 36);
    renderer->quadModel = generateVertexBuffer(global_quadData, 4, global_quadIndices, 6, ATTRIB_INSTANCE_TYPE_MODEL_MATRIX);
    renderer->blockModelWithInstancedT = generateVertexBuffer(global_cubeData, 24, global_cubeIndices, 36, ATTRIB_INSTANCE_TYPE_MODEL_MATRIX);
    renderer->blockModelSameTexture = generateVertexBuffer(global_cubeData_sameTexture, 24, global_cubeIndices, 36, ATTRIB_INSTANCE_TYPE_MODEL_MATRIX);

    return renderer;
}

#include "./sound.cpp"
#include "./perlin.cpp"
#include "./SimplexNoise.cpp"
#include "./interaction.cpp"
#include "./texture_atlas.cpp"
#include "./gameState.cpp"
#include "./chunk.cpp"
#include "./player.cpp"
#include "./camera.cpp"
#include "./perlin_noise_test.cpp"
#include "./imgui.cpp"
#include "./minecraft_animations.cpp"
#include "./animation.cpp"


TimeOfDayValues getTimeOfDayValues(GameState *gameState) {
    float4 a;
    float4 b;

    float4 dayA = make_float4(0.678, 0.847, 0.901, 1);
    float4 dayB = make_float4(0.126, 0.162, 0.529, 1);
    
    TimeOfDayValues result = {};
    result.skyColorA = dayA;
    result.skyColorB = dayB;

    return result;
}

void updateGame(GameState *gameState) {
    if(!gameState->inited) {
        globalLongTermArena = createArena(Kilobytes(200));
        globalPerFrameArena = createArena(Kilobytes(100));
        perFrameArenaMark = takeMemoryMark(&globalPerFrameArena);
        initGameState(gameState);
    } else { 
        releaseMemoryMark(&perFrameArenaMark);
        perFrameArenaMark = takeMemoryMark(&globalPerFrameArena);
    }

    // updateCamera(gameState);
  
    float16 screenGuiT = make_ortho_matrix_origin_center(100, 100*gameState->aspectRatio_y_over_x, MATH_3D_NEAR_CLIP_PlANE, MATH_3D_FAR_CLIP_PlANE);
    float16 textGuiT = make_ortho_matrix_top_left_corner_y_down(100, 100*gameState->aspectRatio_y_over_x, MATH_3D_NEAR_CLIP_PlANE, MATH_3D_FAR_CLIP_PlANE);

    float16 screenT = make_perspective_matrix_origin_center(gameState->camera.fov, MATH_3D_NEAR_CLIP_PlANE, MATH_3D_FAR_CLIP_PlANE, 1.0f / gameState->aspectRatio_y_over_x);
    float16 cameraT = getCameraX(gameState->camera.T);
    float16 cameraTWithoutTranslation = getCameraX_withoutTranslation(gameState->camera.T);

    float16 rot = eulerAnglesToTransform(gameState->camera.T.rotation.y, gameState->camera.T.rotation.x, gameState->camera.T.rotation.z);
    float3 lookingAxis = make_float3(rot.E_[2][0], rot.E_[2][1], rot.E_[2][2]);

    gameState->physicsAccum += gameState->dt;

    float minStep = 0.01f;

    for(int i = 0; i < gameState->voxelEntityCount; ++i) {
        VoxelEntity *e = &gameState->voxelEntities[i];
        e->ddPForFrame = make_float3(0, 0, 0);
        if(e->inverseMass > 0) {
            e->ddPForFrame.y -= 10.0f; //NOTE: Gravity
        }
    }

    //NOTE: Physics loop
    while(gameState->physicsAccum >= minStep) {
        float dt = minStep;

        for(int i = 0; i < gameState->voxelEntityCount; ++i) {
            VoxelEntity *e = &gameState->voxelEntities[i];

            for(int j = i + 1; j < gameState->voxelEntityCount; ++j) {
                VoxelEntity *e1 = &gameState->voxelEntities[j];

                if(e != e1) {
                    // 1st - check bounding box intersection
                    // 2nd -  

                    if (!(e->inverseMass == 0.0f && e1->inverseMass == 0.0f)) {
                        collideVoxelEntities(&gameState->physicsWorld, e, e1); 
                    }
                    
                } else {
                    assert(false);
                }
            }
        }

        //NOTE: Integrate forces
        for(int i = 0; i < gameState->voxelEntityCount; ++i) {
            VoxelEntity *e = &gameState->voxelEntities[i];
            
            e->dP = plus_float3(e->dP, scale_float3(dt, e->ddPForFrame));

            float size = float2_magnitude(e->dP.xy);
            
            if(size < PHYSICS_VELOCITY_SLEEP_THRESHOLD) {
                if(!e->asleep) {
                    e->sleepTimer += dt;
                }
            } else {
                //NOTE: Make sure the entity is awake
                e->sleepTimer = 0;
                e->asleep = false;
            }

            if(e->sleepTimer >= PHYSICS_SLEEP_TIME_THRESHOLD) {
                //NOTE: Object is asleep now
                // e->asleep = true;
            }
        }

        //NOTE: Calculate constant values and conditions the iteration solver should use
        prestepAllArbiters(&gameState->physicsWorld, 1.0f / dt);

        //NOTE: Apply impluses
        updateAllArbiters(&gameState->physicsWorld);

        //NOTE: Integrate velocities
        for(int i = 0; i < gameState->voxelEntityCount; ++i) {
            VoxelEntity *e = &gameState->voxelEntities[i];

            // if(!e->asleep) 
            {
                //NOTE: Object isn't asleep
                e->T.pos = plus_float3(e->T.pos, scale_float3(dt, e->dP));
            }
        }

        gameState->physicsAccum -= minStep;
    }

    for(int i = 0; i < gameState->voxelEntityCount; ++i) {
        VoxelEntity *e = &gameState->voxelEntities[i];

        for(int y = 0; y < e->pitch; ++y) {
            for(int x = 0; x < e->stride; ++x) {
                u8 state = e->data[y*e->stride + x];

                if(state & VOXEL_OCCUPIED) 
                {
                    float4 color = make_float4(1, 0.5f, 0, 1);
                    if(state & VOXEL_COLLIDING) {
                        color = make_float4(0, 0.5f, 1, 1);
                    } else if(e->asleep) {
                        color = make_float4(0, 0, 1, 1);
                    }

                    float3 p = voxelToWorldP(e, x, y);
                    pushColoredQuad(gameState->renderer, p, make_float2(VOXEL_SIZE_IN_METERS, VOXEL_SIZE_IN_METERS), color);
                }
            }
        }
    }

    TimeOfDayValues timeOfDayValues = getTimeOfDayValues(gameState);
    rendererFinish(gameState->renderer, screenT, cameraT, screenGuiT, textGuiT, lookingAxis, cameraTWithoutTranslation, timeOfDayValues, gameState->perlinTestTexture.handle);

    if(gameState->mouseLeftBtn == MOUSE_BUTTON_PRESSED) {
        float2 p = scale_float2(3.0f, getPlaneSize(gameState->camera.fov, 1.0f / gameState->aspectRatio_y_over_x));
        float x = lerp(-p.x, p.x, make_lerpTValue(gameState->mouseP_01.x));
        float y = lerp(-p.y, p.y, make_lerpTValue(1.0f + gameState->mouseP_01.y));
        gameState->voxelEntities[gameState->voxelEntityCount++] = createVoxelCircleEntity(1, make_float3(x, y, 0), 1.0f / 1.0f, gameState->randomStartUpID);
    }
}