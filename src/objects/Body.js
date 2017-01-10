import EventTarget from '../utils/EventTarget';
import Shape from '../shapes/Shape';
import Vec3 from '../math/Vec3';
import Mat3 from '../math/Mat3';
import Quaternion from '../math/Quaternion';
import Material from '../material/Material';
import AABB from '../collision/AABB';
import Box from '../shapes/Box';

/**
 * Base class for all body types.
 * @class Body
 * @constructor
 * @extends EventTarget
 * @param {object} [options]
 * @param {Vec3} [options.position]
 * @param {Vec3} [options.velocity]
 * @param {Vec3} [options.angularVelocity]
 * @param {Quaternion} [options.quaternion]
 * @param {number} [options.mass]
 * @param {Material} [options.material]
 * @param {number} [options.type]
 * @param {number} [options.linearDamping=0.01]
 * @param {number} [options.angularDamping=0.01]
 * @param {boolean} [options.allowSleep=true]
 * @param {number} [options.sleepSpeedLimit=0.1]
 * @param {number} [options.sleepTimeLimit=1]
 * @param {number} [options.collisionFilterGroup=1]
 * @param {number} [options.collisionFilterMask=1]
 * @param {boolean} [options.fixedRotation=false]
 * @param {Body} [options.shape]
 * @example
 *     var body = new Body({
 *         mass: 1
 *     });
 *     var shape = new Sphere(1);
 *     body.addShape(shape);
 *     world.add(body);
 */
class Body extends EventTarget {
 constructor(options={}) {
    super();

  this.id = Body.idCounter++;

  /**
   * Reference to the world the body is living in
   * @property world
   * @type {World}
   */
  this.world = null;

  /**
   * Callback function that is used BEFORE stepping the system. Use it to apply forces, for example. Inside the function, "this" will refer to this Body object.
   * @property preStep
   * @type {Function}
   * @deprecated Use World events instead
   */
  this.preStep = null;

  /**
   * Callback function that is used AFTER stepping the system. Inside the function, "this" will refer to this Body object.
   * @property postStep
   * @type {Function}
   * @deprecated Use World events instead
   */
  this.postStep = null;

  this.vlambda = new Vec3();

  /**
   * @property {Number} collisionFilterGroup
   */
  this.collisionFilterGroup = typeof(options.collisionFilterGroup) === 'number' ? options.collisionFilterGroup : 1;

  /**
   * @property {Number} collisionFilterMask
   */
  this.collisionFilterMask = typeof(options.collisionFilterMask) === 'number' ? options.collisionFilterMask : 1;

  /**
   * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled.
   * @property {Number} collisionResponse
   */
  this.collisionResponse = true;

  /**
   * @property position
   * @type {Vec3}
   */
  this.position = new Vec3();

  if(options.position){
      this.position.copy(options.position);
  }

  /**
   * @property {Vec3} previousPosition
   */
  this.previousPosition = new Vec3();

  /**
   * Initial position of the body
   * @property initPosition
   * @type {Vec3}
   */
  this.initPosition = new Vec3();

  /**
   * @property velocity
   * @type {Vec3}
   */
  this.velocity = new Vec3();

  if(options.velocity){
      this.velocity.copy(options.velocity);
  }

  /**
   * @property initVelocity
   * @type {Vec3}
   */
  this.initVelocity = new Vec3();

  /**
   * Linear force on the body
   * @property force
   * @type {Vec3}
   */
  this.force = new Vec3();

  const mass = typeof(options.mass) === 'number' ? options.mass : 0;

  /**
   * @property mass
   * @type {Number}
   * @default 0
   */
  this.mass = mass;

  /**
   * @property invMass
   * @type {Number}
   */
  this.invMass = mass > 0 ? 1.0 / mass : 0;

  /**
   * @property material
   * @type {Material}
   */
  this.material = options.material || null;

  /**
   * @property linearDamping
   * @type {Number}
   */
  this.linearDamping = typeof(options.linearDamping) === 'number' ? options.linearDamping : 0.01;

  /**
   * One of: Body.DYNAMIC, Body.STATIC and Body.KINEMATIC.
   * @property type
   * @type {Number}
   */
  this.type = (mass <= 0.0 ? Body.STATIC : Body.DYNAMIC);
  if(typeof(options.type) === typeof(Body.STATIC)){
      this.type = options.type;
  }

  /**
   * If true, the body will automatically fall to sleep.
   * @property allowSleep
   * @type {Boolean}
   * @default true
   */
  this.allowSleep = typeof(options.allowSleep) !== 'undefined' ? options.allowSleep : true;

  /**
   * Current sleep state.
   * @property sleepState
   * @type {Number}
   */
  this.sleepState = 0;

  /**
   * If the speed (the norm of the velocity) is smaller than this value, the body is considered sleepy.
   * @property sleepSpeedLimit
   * @type {Number}
   * @default 0.1
   */
  this.sleepSpeedLimit = typeof(options.sleepSpeedLimit) !== 'undefined' ? options.sleepSpeedLimit : 0.1;

  /**
   * If the body has been sleepy for this sleepTimeLimit seconds, it is considered sleeping.
   * @property sleepTimeLimit
   * @type {Number}
   * @default 1
   */
  this.sleepTimeLimit = typeof(options.sleepTimeLimit) !== 'undefined' ? options.sleepTimeLimit : 1;

  this.timeLastSleepy = 0;

  this._wakeUpAfterNarrowphase = false;


  /**
   * Rotational force on the body, around center of mass
   * @property {Vec3} torque
   */
  this.torque = new Vec3();

  /**
   * Orientation of the body
   * @property quaternion
   * @type {Quaternion}
   */
  this.quaternion = new Quaternion();

  if(options.quaternion){
      this.quaternion.copy(options.quaternion);
  }

  /**
   * @property initQuaternion
   * @type {Quaternion}
   */
  this.initQuaternion = new Quaternion();

  /**
   * @property angularVelocity
   * @type {Vec3}
   */
  this.angularVelocity = new Vec3();

  if(options.angularVelocity){
      this.angularVelocity.copy(options.angularVelocity);
  }

  /**
   * @property initAngularVelocity
   * @type {Vec3}
   */
  this.initAngularVelocity = new Vec3();

  this.interpolatedPosition = new Vec3();
  this.interpolatedQuaternion = new Quaternion();

  /**
   * @property shapes
   * @type {array}
   */
  this.shapes = [];

  /**
   * @property shapeOffsets
   * @type {array}
   */
  this.shapeOffsets = [];

  /**
   * @property shapeOrientations
   * @type {array}
   */
  this.shapeOrientations = [];

  /**
   * @property inertia
   * @type {Vec3}
   */
  this.inertia = new Vec3();

  /**
   * @property {Vec3} invInertia
   */
  this.invInertia = new Vec3();

  /**
   * @property {Mat3} invInertiaWorld
   */
  this.invInertiaWorld = new Mat3();

  this.invMassSolve = 0;

  /**
   * @property {Vec3} invInertiaSolve
   */
  this.invInertiaSolve = new Vec3();

  /**
   * @property {Mat3} invInertiaWorldSolve
   */
  this.invInertiaWorldSolve = new Mat3();

  /**
   * Set to true if you don't want the body to rotate. Make sure to run .updateMassProperties() after changing this.
   * @property {Boolean} fixedRotation
   * @default false
   */
  this.fixedRotation = typeof(options.fixedRotation) !== "undefined" ? options.fixedRotation : false;

  /**
   * @property {Number} angularDamping
   */
  this.angularDamping = typeof(options.angularDamping) !== 'undefined' ? options.angularDamping : 0.01;

  /**
   * @property aabb
   * @type {AABB}
   */
  this.aabb = new AABB();

  /**
   * Indicates if the AABB needs to be updated before use.
   * @property aabbNeedsUpdate
   * @type {Boolean}
   */
  this.aabbNeedsUpdate = true;

  this.wlambda = new Vec3();

  if(options.shape){
      this.addShape(options.shape);
  }

  this.updateMassProperties();
 }

 /**
  * Wake the body up.
  * @method wakeUp
  */
 wakeUp() {
     const s = this.sleepState;
     this.sleepState = 0;
     if(s === Body.SLEEPING){
         this.dispatchEvent({type:"wakeup"});
     }
 }

 /**
  * Force body sleep
  * @method sleep
  */
 sleep() {
     this.sleepState = Body.SLEEPING;
     this.velocity.set(0,0,0);
     this.angularVelocity.set(0,0,0);
 }

 /**
  * Called every timestep to update internal sleep timer and change sleep state if needed.
  * @method sleepTick
  * @param {Number} time The world time in seconds
  */
 sleepTick(time) {
     if(this.allowSleep){
         const sleepState = this.sleepState;
         const speedSquared = this.velocity.norm2() + this.angularVelocity.norm2();
         const speedLimitSquared = Math.pow(this.sleepSpeedLimit, 2);
         if(sleepState===Body.AWAKE && speedSquared < speedLimitSquared){
             this.sleepState = Body.SLEEPY; // Sleepy
             this.timeLastSleepy = time;
             this.dispatchEvent(Body.sleepyEvent);
         } else if(sleepState===Body.SLEEPY && speedSquared > speedLimitSquared){
             this.wakeUp(); // Wake up
         } else if(sleepState===Body.SLEEPY && (time - this.timeLastSleepy ) > this.sleepTimeLimit){
             this.sleep(); // Sleeping
             this.dispatchEvent(Body.sleepEvent);
         }
     }
 }

 /**
  * If the body is sleeping, it should be immovable / have infinite mass during solve. We solve it by having a separate "solve mass".
  * @method updateSolveMassProperties
  */
 updateSolveMassProperties() {
     if(this.sleepState === Body.SLEEPING || this.type === Body.KINEMATIC){
         this.invMassSolve = 0;
         this.invInertiaSolve.setZero();
         this.invInertiaWorldSolve.setZero();
     } else {
         this.invMassSolve = this.invMass;
         this.invInertiaSolve.copy(this.invInertia);
         this.invInertiaWorldSolve.copy(this.invInertiaWorld);
     }
 }

 /**
  * Convert a world point to local body frame.
  * @method pointToLocalFrame
  * @param  {Vec3} worldPoint
  * @param  {Vec3} result
  * @return {Vec3}
  */
 pointToLocalFrame(worldPoint, result) {
     var result = result || new Vec3();
     worldPoint.vsub(this.position,result);
     this.quaternion.conjugate().vmult(result,result);
     return result;
 }

 /**
  * Convert a world vector to local body frame.
  * @method vectorToLocalFrame
  * @param  {Vec3} worldPoint
  * @param  {Vec3} result
  * @return {Vec3}
  */
 vectorToLocalFrame(worldVector, result) {
     var result = result || new Vec3();
     this.quaternion.conjugate().vmult(worldVector,result);
     return result;
 }

 /**
  * Convert a local body point to world frame.
  * @method pointToWorldFrame
  * @param  {Vec3} localPoint
  * @param  {Vec3} result
  * @return {Vec3}
  */
 pointToWorldFrame(localPoint, result) {
     var result = result || new Vec3();
     this.quaternion.vmult(localPoint,result);
     result.vadd(this.position,result);
     return result;
 }

 /**
  * Convert a local body point to world frame.
  * @method vectorToWorldFrame
  * @param  {Vec3} localVector
  * @param  {Vec3} result
  * @return {Vec3}
  */
 vectorToWorldFrame(localVector, result) {
     var result = result || new Vec3();
     this.quaternion.vmult(localVector, result);
     return result;
 }

 /**
  * Add a shape to the body with a local offset and orientation.
  * @method addShape
  * @param {Shape} shape
  * @param {Vec3} offset
  * @param {Quaternion} quaternion
  * @return {Body} The body object, for chainability.
  */
 addShape(shape, _offset, _orientation) {
     const offset = new Vec3();
     const orientation = new Quaternion();

     if(_offset){
         offset.copy(_offset);
     }
     if(_orientation){
         orientation.copy(_orientation);
     }

     this.shapes.push(shape);
     this.shapeOffsets.push(offset);
     this.shapeOrientations.push(orientation);
     this.updateMassProperties();
     this.updateBoundingRadius();

     this.aabbNeedsUpdate = true;

     return this;
 }

 /**
  * Update the bounding radius of the body. Should be done if any of the shapes are changed.
  * @method updateBoundingRadius
  */
 updateBoundingRadius() {
  const shapes = this.shapes;
  const shapeOffsets = this.shapeOffsets;
  const N = shapes.length;
  let radius = 0;

  for(let i=0; i!==N; i++){
   const shape = shapes[i];
   shape.updateBoundingSphereRadius();
   const offset = shapeOffsets[i].norm();
   const r = shape.boundingSphereRadius;
   if(offset + r > radius){
       radius = offset + r;
   }
  }

  this.boundingRadius = radius;
 }

 /**
  * Updates the .aabb
  * @method computeAABB
  * @todo rename to updateAABB()
  */
 computeAABB() {
  const shapes = this.shapes;
  const shapeOffsets = this.shapeOffsets;
  const shapeOrientations = this.shapeOrientations;
  const N = shapes.length;
  const offset = tmpVec;
  const orientation = tmpQuat;
  const bodyQuat = this.quaternion;
  const aabb = this.aabb;
  const shapeAABB = computeAABB_shapeAABB;

  for(let i=0; i!==N; i++){
      const shape = shapes[i];

      // Get shape world quaternion
      shapeOrientations[i].mult(bodyQuat, orientation);

      // Get shape world position
      orientation.vmult(shapeOffsets[i], offset);
      offset.vadd(this.position, offset);

      // vec2.rotate(offset, shapeOffsets[i], bodyAngle);
      // vec2.add(offset, offset, this.position);

      // Get shape AABB
      shape.calculateWorldAABB(offset, orientation, shapeAABB.lowerBound, shapeAABB.upperBound);

      if(i === 0){
          aabb.copy(shapeAABB);
      } else {
          aabb.extend(shapeAABB);
      }
  }

  this.aabbNeedsUpdate = false;
 }

 /**
  * Update .inertiaWorld and .invInertiaWorld
  * @method updateInertiaWorld
  */
 updateInertiaWorld(force) {
     const I = this.invInertia;
     if (I.x === I.y && I.y === I.z && !force) {
         // If inertia M = s*I, where I is identity and s a scalar, then
         //    R*M*R' = R*(s*I)*R' = s*R*I*R' = s*R*R' = s*I = M
         // where R is the rotation matrix.
         // In other words, we don't have to transform the inertia if all
         // inertia diagonal entries are equal.
     } else {
      const m1 = uiw_m1;
      const m2 = uiw_m2;
      const m3 = uiw_m3;
      m1.setRotationFromQuaternion(this.quaternion);
      m1.transpose(m2);
      m1.scale(I,m1);
      m1.mmult(m2,this.invInertiaWorld);
      //m3.getTrace(this.invInertiaWorld);
     }

     /*
     this.quaternion.vmult(this.inertia,this.inertiaWorld);
     this.quaternion.vmult(this.invInertia,this.invInertiaWorld);
     */
 }

 applyForce(force, worldPoint) {
     if(this.type !== Body.DYNAMIC){
         return;
     }

     // Compute point position relative to the body center
     const r = Body_applyForce_r;
     worldPoint.vsub(this.position,r);

     // Compute produced rotational force
     const rotForce = Body_applyForce_rotForce;
     r.cross(force,rotForce);

     // Add linear force
     this.force.vadd(force,this.force);

     // Add rotational force
     this.torque.vadd(rotForce,this.torque);
 }

 applyLocalForce(localForce, localPoint) {
     if(this.type !== Body.DYNAMIC){
         return;
     }

     const worldForce = Body_applyLocalForce_worldForce;
     const worldPoint = Body_applyLocalForce_worldPoint;

     // Transform the force vector to world space
     this.vectorToWorldFrame(localForce, worldForce);
     this.pointToWorldFrame(localPoint, worldPoint);

     this.applyForce(worldForce, worldPoint);
 }

 applyImpulse(impulse, worldPoint) {
     if(this.type !== Body.DYNAMIC){
         return;
     }

     // Compute point position relative to the body center
     const r = Body_applyImpulse_r;
     worldPoint.vsub(this.position,r);

     // Compute produced central impulse velocity
     const velo = Body_applyImpulse_velo;
     velo.copy(impulse);
     velo.mult(this.invMass,velo);

     // Add linear impulse
     this.velocity.vadd(velo, this.velocity);

     // Compute produced rotational impulse velocity
     const rotVelo = Body_applyImpulse_rotVelo;
     r.cross(impulse,rotVelo);

     /*
     rotVelo.x *= this.invInertia.x;
     rotVelo.y *= this.invInertia.y;
     rotVelo.z *= this.invInertia.z;
     */
     this.invInertiaWorld.vmult(rotVelo,rotVelo);

     // Add rotational Impulse
     this.angularVelocity.vadd(rotVelo, this.angularVelocity);
 }

 applyLocalImpulse(localImpulse, localPoint) {
     if(this.type !== Body.DYNAMIC){
         return;
     }

     const worldImpulse = Body_applyLocalImpulse_worldImpulse;
     const worldPoint = Body_applyLocalImpulse_worldPoint;

     // Transform the force vector to world space
     this.vectorToWorldFrame(localImpulse, worldImpulse);
     this.pointToWorldFrame(localPoint, worldPoint);

     this.applyImpulse(worldImpulse, worldPoint);
 }

 /**
  * Should be called whenever you change the body shape or mass.
  * @method updateMassProperties
  */
 updateMassProperties() {
     const halfExtents = Body_updateMassProperties_halfExtents;

     this.invMass = this.mass > 0 ? 1.0 / this.mass : 0;
     const I = this.inertia;
     const fixed = this.fixedRotation;

     // Approximate with AABB box
     this.computeAABB();
     halfExtents.set(
         (this.aabb.upperBound.x-this.aabb.lowerBound.x) / 2,
         (this.aabb.upperBound.y-this.aabb.lowerBound.y) / 2,
         (this.aabb.upperBound.z-this.aabb.lowerBound.z) / 2
     );
     Box.calculateInertia(halfExtents, this.mass, I);

     this.invInertia.set(
         I.x > 0 && !fixed ? 1.0 / I.x : 0,
         I.y > 0 && !fixed ? 1.0 / I.y : 0,
         I.z > 0 && !fixed ? 1.0 / I.z : 0
     );
     this.updateInertiaWorld(true);
 }

 /**
  * Get world velocity of a point in the body.
  * @method getVelocityAtWorldPoint
  * @param  {Vec3} worldPoint
  * @param  {Vec3} result
  * @return {Vec3} The result vector.
  */
 getVelocityAtWorldPoint(worldPoint, result) {
     const r = new Vec3();
     worldPoint.vsub(this.position, r);
     this.angularVelocity.cross(r, result);
     this.velocity.vadd(result, result);
     return result;
 }
}

/**
 * A dynamic body is fully simulated. Can be moved manually by the user, but normally they move according to forces. A dynamic body can collide with all body types. A dynamic body always has finite, non-zero mass.
 * @static
 * @property DYNAMIC
 * @type {Number}
 */
Body.DYNAMIC = 1;

/**
 * A static body does not move during simulation and behaves as if it has infinite mass. Static bodies can be moved manually by setting the position of the body. The velocity of a static body is always zero. Static bodies do not collide with other static or kinematic bodies.
 * @static
 * @property STATIC
 * @type {Number}
 */
Body.STATIC = 2;

/**
 * A kinematic body moves under simulation according to its velocity. They do not respond to forces. They can be moved manually, but normally a kinematic body is moved by setting its velocity. A kinematic body behaves as if it has infinite mass. Kinematic bodies do not collide with other static or kinematic bodies.
 * @static
 * @property KINEMATIC
 * @type {Number}
 */
Body.KINEMATIC = 4;



/**
 * @static
 * @property AWAKE
 * @type {number}
 */
Body.AWAKE = 0;

/**
 * @static
 * @property SLEEPY
 * @type {number}
 */
Body.SLEEPY = 1;

/**
 * @static
 * @property SLEEPING
 * @type {number}
 */
Body.SLEEPING = 2;

Body.idCounter = 0;

Body.sleepyEvent = {
    type: "sleepy"
};

Body.sleepEvent = {
    type: "sleep"
};

var tmpVec = new Vec3();
var tmpQuat = new Quaternion();

var computeAABB_shapeAABB = new AABB();

var uiw_m1 = new Mat3();
var uiw_m2 = new Mat3();
var uiw_m3 = new Mat3();

/**
 * Apply force to a world point. This could for example be a point on the Body surface. Applying force this way will add to Body.force and Body.torque.
 * @method applyForce
 * @param  {Vec3} force The amount of force to add.
 * @param  {Vec3} worldPoint A world point to apply the force on.
 */
var Body_applyForce_r = new Vec3();
var Body_applyForce_rotForce = new Vec3();

/**
 * Apply force to a local point in the body.
 * @method applyLocalForce
 * @param  {Vec3} force The force vector to apply, defined locally in the body frame.
 * @param  {Vec3} localPoint A local point in the body to apply the force on.
 */
var Body_applyLocalForce_worldForce = new Vec3();
var Body_applyLocalForce_worldPoint = new Vec3();

/**
 * Apply impulse to a world point. This could for example be a point on the Body surface. An impulse is a force added to a body during a short period of time (impulse = force * time). Impulses will be added to Body.velocity and Body.angularVelocity.
 * @method applyImpulse
 * @param  {Vec3} impulse The amount of impulse to add.
 * @param  {Vec3} worldPoint A world point to apply the force on.
 */
var Body_applyImpulse_r = new Vec3();
var Body_applyImpulse_velo = new Vec3();
var Body_applyImpulse_rotVelo = new Vec3();

/**
 * Apply locally-defined impulse to a local point in the body.
 * @method applyLocalImpulse
 * @param  {Vec3} force The force vector to apply, defined locally in the body frame.
 * @param  {Vec3} localPoint A local point in the body to apply the force on.
 */
var Body_applyLocalImpulse_worldImpulse = new Vec3();
var Body_applyLocalImpulse_worldPoint = new Vec3();

var Body_updateMassProperties_halfExtents = new Vec3();

export default Body;
