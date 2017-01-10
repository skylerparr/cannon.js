import Body from '../objects/Body';
import Vec3 from '../math/Vec3';
import Quaternion from '../math/Quaternion';
import Shape from '../shapes/Shape';
import Plane from '../shapes/Plane';

/**
 * Base class for broadphase implementations
 * @class Broadphase
 * @constructor
 * @author schteppe
 */
class Broadphase {
    constructor() {
        /**
        * The world to search for collisions in.
        * @property world
        * @type {World}
        */
        this.world = null;

        /**
         * If set to true, the broadphase uses bounding boxes for intersection test, else it uses bounding spheres.
         * @property useBoundingBoxes
         * @type {Boolean}
         */
        this.useBoundingBoxes = false;

        /**
         * Set to true if the objects in the world moved.
         * @property {Boolean} dirty
         */
        this.dirty = true;
    }

    /**
     * Check if the bounding volumes of two bodies intersect.
     * @method intersectionTest
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {array} pairs1
     * @param {array} pairs2
      */
    intersectionTest(bodyA, bodyB, pairs1, pairs2) {
        if(this.useBoundingBoxes){
            this.doBoundingBoxBroadphase(bodyA,bodyB,pairs1,pairs2);
        } else {
            this.doBoundingSphereBroadphase(bodyA,bodyB,pairs1,pairs2);
        }
    }
}

/**
 * Get the collision pairs from the world
 * @method collisionPairs
 * @param {World} world The world to search in
 * @param {Array} p1 Empty array to be filled with body objects
 * @param {Array} p2 Empty array to be filled with body objects
 */
Broadphase.prototype.collisionPairs = (world, p1, p2) => {
    throw new Error("collisionPairs not implemented for this BroadPhase class!");
};

/**
 * Check if a body pair needs to be intersection tested at all.
 * @method needBroadphaseCollision
 * @param {Body} bodyA
 * @param {Body} bodyB
 * @return {bool}
 */
const Broadphase_needBroadphaseCollision_STATIC_OR_KINEMATIC = Body.STATIC | Body.KINEMATIC;
Broadphase.prototype.needBroadphaseCollision = (bodyA, bodyB) => {

    // Check collision filter masks
    if( (bodyA.collisionFilterGroup & bodyB.collisionFilterMask)===0 || (bodyB.collisionFilterGroup & bodyA.collisionFilterMask)===0){
        return false;
    }

    // Check types
    if(((bodyA.type & Broadphase_needBroadphaseCollision_STATIC_OR_KINEMATIC)!==0 || bodyA.sleepState === Body.SLEEPING) &&
       ((bodyB.type & Broadphase_needBroadphaseCollision_STATIC_OR_KINEMATIC)!==0 || bodyB.sleepState === Body.SLEEPING)) {
        // Both bodies are static, kinematic or sleeping. Skip.
        return false;
    }

    return true;
};

/**
 * Check if the bounding spheres of two bodies are intersecting.
 * @method doBoundingSphereBroadphase
 * @param {Body} bodyA
 * @param {Body} bodyB
 * @param {Array} pairs1 bodyA is appended to this array if intersection
 * @param {Array} pairs2 bodyB is appended to this array if intersection
 */
const // Temp objects
Broadphase_collisionPairs_r = new Vec3();

const Broadphase_collisionPairs_normal =  new Vec3();
const Broadphase_collisionPairs_quat =  new Quaternion();
const Broadphase_collisionPairs_relpos  =  new Vec3();
Broadphase.prototype.doBoundingSphereBroadphase = (bodyA, bodyB, pairs1, pairs2) => {
    const r = Broadphase_collisionPairs_r;
    bodyB.position.vsub(bodyA.position,r);
    const boundingRadiusSum2 = Math.pow(bodyA.boundingRadius + bodyB.boundingRadius, 2);
    const norm2 = r.norm2();
    if(norm2 < boundingRadiusSum2){
        pairs1.push(bodyA);
        pairs2.push(bodyB);
    }
};

/**
 * Check if the bounding boxes of two bodies are intersecting.
 * @method doBoundingBoxBroadphase
 * @param {Body} bodyA
 * @param {Body} bodyB
 * @param {Array} pairs1
 * @param {Array} pairs2
 */
Broadphase.prototype.doBoundingBoxBroadphase = (bodyA, bodyB, pairs1, pairs2) => {
    if(bodyA.aabbNeedsUpdate){
        bodyA.computeAABB();
    }
    if(bodyB.aabbNeedsUpdate){
        bodyB.computeAABB();
    }

    // Check AABB / AABB
    if(bodyA.aabb.overlaps(bodyB.aabb)){
        pairs1.push(bodyA);
        pairs2.push(bodyB);
    }
};

/**
 * Removes duplicate pairs from the pair arrays.
 * @method makePairsUnique
 * @param {Array} pairs1
 * @param {Array} pairs2
 */
const Broadphase_makePairsUnique_temp = { keys:[] };

const Broadphase_makePairsUnique_p1 = [];
const Broadphase_makePairsUnique_p2 = [];
Broadphase.prototype.makePairsUnique = (pairs1, pairs2) => {
    const t = Broadphase_makePairsUnique_temp;
    const p1 = Broadphase_makePairsUnique_p1;
    const p2 = Broadphase_makePairsUnique_p2;
    const N = pairs1.length;

    for(var i=0; i!==N; i++){
        p1[i] = pairs1[i];
        p2[i] = pairs2[i];
    }

    pairs1.length = 0;
    pairs2.length = 0;

    for(var i=0; i!==N; i++){
        const id1 = p1[i].id;
        const id2 = p2[i].id;
        var key = id1 < id2 ? `${id1},${id2}` :  `${id2},${id1}`;
        t[key] = i;
        t.keys.push(key);
    }

    for(var i=0; i!==t.keys.length; i++){
        const key = t.keys.pop();
        const pairIndex = t[key];
        pairs1.push(p1[pairIndex]);
        pairs2.push(p2[pairIndex]);
        delete t[key];
    }
};

/**
 * To be implemented by subcasses
 * @method setWorld
 * @param {World} world
 */
Broadphase.prototype.setWorld = world => {
};

/**
 * Check if the bounding spheres of two bodies overlap.
 * @method boundingSphereCheck
 * @param {Body} bodyA
 * @param {Body} bodyB
 * @return {boolean}
 */
const bsc_dist = new Vec3();
Broadphase.boundingSphereCheck = (bodyA, bodyB) => {
    const dist = bsc_dist;
    bodyA.position.vsub(bodyB.position,dist);
    return Math.pow(bodyA.shape.boundingSphereRadius + bodyB.shape.boundingSphereRadius, 2) > dist.norm2();
};

/**
 * Returns all the bodies within the AABB.
 * @method aabbQuery
 * @param  {World} world
 * @param  {AABB} aabb
 * @param  {array} result An array to store resulting bodies in.
 * @return {array}
 */
Broadphase.prototype.aabbQuery = (world, aabb, result) => {
    console.warn('.aabbQuery is not implemented in this Broadphase subclass.');
    return [];
};

export default Broadphase;
