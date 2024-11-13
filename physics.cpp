struct CollisionPoint {
    //NOTE: Data
    float2 point;
    float2 normal;
    float inverseMassNormal; //NOTE: The dividend of the J calculation. This is constant throughout the iterations
    float velocityBias; //NOTE: This is an added velocity to stop penetration along the collision normal
    float seperation; //NOTE: Negative if shapes are colliding. 
    float Pn; //NOTE: Accumulated normal impulse

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
    bool positionCorrecting; //DONE: this is how agressively the physics tries and resolves penetration
    bool accumulateImpulses;

    //NOTE: Erin Catto's talk at GDC 2006

    //NEXT: Add angular momentum in now
};

void prestepAllArbiters(PhysicsWorld *world, float inverseDt) {
    Arbiter *arb = world->arbiters;

    float allowedPenertration = 0.01;//NOTE: Meters

    //NOTE: If positionCorrection correction is on
    // The bias factor is important because overly aggressive corrections (with a high bias factor) can cause instability or jittering in the simulation, while too small of a correction (with a low bias factor) may leave objects slightly penetrated.
	// It strikes a balance between stability and realism, ensuring that objects resolve overlaps without visibly popping or jittering in the simulation.
    float biasFactor = (world->positionCorrecting) ? 1.0f : 0.0f;

    while(arb) {
        float massCombined = arb->a->inverseMass + arb->b->inverseMass;

        for(int i = 0; i < arb->pointsCount; i++) {
            CollisionPoint *p = &arb->points[i];

            p->velocityBias = -biasFactor * MathMinf((p->seperation + allowedPenertration), 0) * inverseDt;
            p->inverseMassNormal = 1.0f / float2_dot(p->normal, scale_float2(massCombined, p->normal));

            if (world->accumulateImpulses) {
                //NOTE: The longer this point is colliding the bigger the impulse is applied to it
                // Apply normal + friction impulse
                float2 Pn = scale_float2(p->Pn, p->normal);
                arb->a->dP.xy = minus_float2(arb->a->dP.xy, scale_float2(arb->a->inverseMass, Pn));
                arb->b->dP.xy = plus_float2(arb->b->dP.xy, scale_float2(arb->b->inverseMass, Pn));
            }
        }

        arb = arb->next;
    }
}

void updateAllArbiters(PhysicsWorld *world) {
    Arbiter *arb = world->arbiters;

    const int iterationCount = 10;

    int arbCount = 0;

    while(arb) {
        arbCount++;
        for(int i = 0; i < iterationCount; i++) {
            for(int i = 0; i < arb->pointsCount; i++) {
                CollisionPoint *p = &arb->points[i];

                float e = MathMaxf(arb->a->coefficientOfRestitution, arb->b->coefficientOfRestitution);

                const float2 velocityA = arb->a->dP.xy;
                const float2 velocityB = arb->b->dP.xy;

                float2 relativeAB = minus_float2(velocityB, velocityA);

                if(float2_dot(relativeAB, relativeAB) < (PHYSICS_RESTITUTION_VELOCITY_THRESHOLD_SQR)) {
                    e = 0;
                }

                float vn = float2_dot(relativeAB, p->normal);

                //NOTE: This is the J calculation as in Chris Hecker's phsyics tutorials
                float dPn = ((-(1 + e)*vn) + p->velocityBias) * p->inverseMassNormal;

                if(world->accumulateImpulses) {
                    assert(p->Pn >= 0);
                    float Pn0 = p->Pn; //NOTE: Impulse magnitude previous frame
                    p->Pn = MathMaxf(Pn0 + dPn, 0.0f);
                    dPn = p->Pn - Pn0;
                    //NOTE: This allows impulses to go towards the collision points
                } else {
                    //NOTE: Moving towards the objects so remove velocities component that's contributing to them colliding more
                    dPn = MathMaxf(dPn, 0.0f);
                }

                float2 Pn = scale_float2(dPn, p->normal);

                arb->a->dP.xy = minus_float2(arb->a->dP.xy, scale_float2(arb->a->inverseMass, Pn));
                arb->b->dP.xy = plus_float2(arb->b->dP.xy, scale_float2(arb->b->inverseMass, Pn));
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

        
        for(int j = 0; j < pointCount; j++) {
            CollisionPoint *newP = &points[j];
            bool pointFound = false;
            for(int i = 0; i < arb->pointsCount && !pointFound; i++) {
                CollisionPoint *oldP = &arb->points[i];
            
                if(oldP->x == newP->x && oldP->y == newP->y &&  areEntityIdsEqual(oldP->entityId, newP->entityId)) {
                    //NOTE: Point already exists, so keep the accumulated impluses going
                    if (world->warmStarting) {
                        newP->Pn = oldP->Pn;
                        // c->Pt = cOld->Pt;
                        // c->Pnb = cOld->Pnb;
                    }
                    pointFound = true;
                }
            }
        }

        assert(pointCount < arrayCount(arb->points));
        arb->pointsCount = pointCount;

        //NOTE: Copy the new points array to the old points array
        for(int i = 0; i < pointCount; i++) {
            arb->points[i] = points[i];
        }
    }
}