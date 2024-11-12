struct CollisionPoint {
    //NOTE: Data
    float2 point;
    float2 normal;

    //NOTE: Used for the id
    EntityID entityId;
    int x;
    int y;
};

struct Arbiter {
    VoxelEntity *a;
    VoxelEntity *b;
    int pointsCount;
    CollisionPoint points[MAX_CONTACT_POINTS_PER_PAIR];

    Arbiter *next;
};

struct PhysicsWorld {
    Arbiter *arbiters;
    Arbiter *arbitersFreeList;

    //TODO: This is what Box2d-lite does to improve it's stability and robustness.
    bool warmStarting;
    bool positionCorrecting;
    bool accumulateImpulses;
};

void updateAllArbiters(PhysicsWorld *world) {
    Arbiter *arb = world->arbiters;

    const int iterationCount = 10;

    int arbCount = 0;

    while(arb) {
        arbCount++;
        float massCombined = arb->a->inverseMass + arb->b->inverseMass;
        for(int i = 0; i < iterationCount; i++) {
            for(int i = 0; i < arb->pointsCount; i++) {
                CollisionPoint *p = &arb->points[i];

                float e = MathMaxf(arb->a->coefficientOfRestitution, arb->b->coefficientOfRestitution);

                const float2 velocityA = arb->a->dP.xy;
                const float2 velocityB = arb->b->dP.xy;

                float2 relativeAB = minus_float2(velocityA, velocityB);
                
                float relVelAlongNormal = float2_dot(relativeAB, p->normal);
                if(relVelAlongNormal < 0) {
                    //NOTE: Moving towards the objects so remove velocities component that's contributing to them colliding more
                    relativeAB = minus_float2(relativeAB, scale_float2(relVelAlongNormal, p->normal));
                }
                if(float2_dot(relativeAB, relativeAB) < (PHYSICS_RESTITUTION_VELOCITY_THRESHOLD_SQR)) {
                    e = 0;
                }

                float J = (float2_dot(scale_float2(-(1 + e), relativeAB), p->normal)) / float2_dot(p->normal, scale_float2(massCombined, p->normal));

                arb->a->dP.xy = plus_float2(velocityA, scale_float2(arb->a->inverseMass*J, p->normal));
                arb->b->dP.xy = plus_float2(velocityB, scale_float2(arb->b->inverseMass*-J, p->normal));
            }
        }
        
        arb = arb->next;
    }

}

void wakeUpEntity(VoxelEntity *e) {
    e->asleep = false;
    e->sleepTimer = 0;
} 

void mergePointsToArbiter(PhysicsWorld *world, CollisionPoint *points, int pointCount, VoxelEntity *a, VoxelEntity *b) {
    if(a > b) {
        VoxelEntity *temp = b;
        b = a;
        a = temp;
    }

    Arbiter *arb = world->arbiters;
    Arbiter **arbPrev = &world->arbiters;
    while(arb) {
        if(arb->a == a && arb->b == b) {
            if(pointCount == 0) {
                //NOTE: Remove from the list
                *arbPrev = arb->next;
                arb->next = world->arbitersFreeList;
                world->arbitersFreeList = arb;
            }
            break;
        } else {
            arbPrev = &arb->next;
            arb = arb->next;
        }
    }

    if(pointCount == 0) {
        
    } else {
        
        if(!arb) {
            if(world->arbitersFreeList) {
                arb = world->arbitersFreeList;
                world->arbitersFreeList = arb->next;
            } else {
                arb = pushStruct(&globalLongTermArena, Arbiter);
            }
            arb->next = world->arbiters;
            world->arbiters = arb;

            arb->pointsCount = 0;

            arb->a = a;
            arb->b = b;
        }

        assert(arb);
        assert(arb->a);
        assert(arb->b);

        
        // for(int j = 0; j < pointCount; j++) {
        //     CollisionPoint *newP = &points[j];
        //     bool pointFound = false;
        //     for(int i = 0; i < arb->pointsCount && !pointFound; i++) {
        //         CollisionPoint *oldP = &arb->points[i];
            
        //         if(oldP->x == newP->x && oldP->y == newP->y &&  areEntityIdsEqual(oldP->entityId, newP->entityId)) {
        //             //NOTE: Point already exists, so copy the info across
        //             // *newP = *oldP;
        //             pointFound = true;
        //         }
        //     }
        // }

        assert(pointCount < arrayCount(arb->points));
        arb->pointsCount = pointCount;

        //NOTE: Copy the new points array to the old points array
        for(int i = 0; i < pointCount; i++) {
            arb->points[i] = points[i];
        }
    }
}