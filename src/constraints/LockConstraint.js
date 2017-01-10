import Constraint from './Constraint';
import PointToPointConstraint from './PointToPointConstraint';
import RotationalEquation from '../equations/RotationalEquation';
import RotationalMotorEquation from '../equations/RotationalMotorEquation';
import ContactEquation from '../equations/ContactEquation';
import Vec3 from '../math/Vec3';

/**
 * Lock constraint. Will remove all degrees of freedom between the bodies.
 * @class LockConstraint
 * @constructor
 * @author schteppe
 * @param {Body} bodyA
 * @param {Body} bodyB
 * @param {object} [options]
 * @param {Number} [options.maxForce=1e6]
 * @extends PointToPointConstraint
 */
class LockConstraint extends PointToPointConstraint{
 constructor(bodyA, bodyB, options={}) {
  const maxForce = typeof(options.maxForce) !== 'undefined' ? options.maxForce : 1e6;

  // Set pivot point in between
  const pivotA = new Vec3();
  const pivotB = new Vec3();
  const halfWay = new Vec3();
  bodyA.position.vadd(bodyB.position, halfWay);
  halfWay.scale(0.5, halfWay);
  bodyB.pointToLocalFrame(halfWay, pivotB);
  bodyA.pointToLocalFrame(halfWay, pivotA);
  super(bodyA, pivotA, bodyB, pivotB, maxForce);

  /**
   * @property {RotationalEquation} rotationalEquation1
   */
  const r1 = this.rotationalEquation1 = new RotationalEquation(bodyA,bodyB,options);

  /**
   * @property {RotationalEquation} rotationalEquation2
   */
  const r2 = this.rotationalEquation2 = new RotationalEquation(bodyA,bodyB,options);

  /**
   * @property {RotationalEquation} rotationalEquation3
   */
  const r3 = this.rotationalEquation3 = new RotationalEquation(bodyA,bodyB,options);

  this.equations.push(r1, r2, r3);
 }

 update() {
  const bodyA = this.bodyA;
  const bodyB = this.bodyB;
  const motor = this.motorEquation;
  const r1 = this.rotationalEquation1;
  const r2 = this.rotationalEquation2;
  const r3 = this.rotationalEquation3;
  const worldAxisA = LockConstraint_update_tmpVec1;
  const worldAxisB = LockConstraint_update_tmpVec2;

  PointToPointConstraint.prototype.update.call(this);

  bodyA.vectorToWorldFrame(Vec3.UNIT_X, r1.axisA);
  bodyB.vectorToWorldFrame(Vec3.UNIT_Y, r1.axisB);

  bodyA.vectorToWorldFrame(Vec3.UNIT_Y, r2.axisA);
  bodyB.vectorToWorldFrame(Vec3.UNIT_Z, r2.axisB);

  bodyA.vectorToWorldFrame(Vec3.UNIT_Z, r3.axisA);
  bodyB.vectorToWorldFrame(Vec3.UNIT_X, r3.axisB);
 }
}

var LockConstraint_update_tmpVec1 = new Vec3();
var LockConstraint_update_tmpVec2 = new Vec3();

export default LockConstraint;
