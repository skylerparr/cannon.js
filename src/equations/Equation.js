import JacobianElement from '../math/JacobianElement';
import Vec3 from '../math/Vec3';

/**
 * Equation base class
 * @class Equation
 * @constructor
 * @author schteppe
 * @param {Body} bi
 * @param {Body} bj
 * @param {Number} minForce Minimum (read: negative max) force to be applied by the constraint.
 * @param {Number} maxForce Maximum (read: positive max) force to be applied by the constraint.
 */
class Equation {
    constructor(bi, bj, minForce, maxForce) {
        this.id = Equation.id++;

        /**
         * @property {number} minForce
         */
        this.minForce = typeof(minForce)==="undefined" ? -1e6 : minForce;

        /**
         * @property {number} maxForce
         */
        this.maxForce = typeof(maxForce)==="undefined" ? 1e6 : maxForce;

        /**
         * @property bi
         * @type {Body}
         */
        this.bi = bi;

        /**
         * @property bj
         * @type {Body}
         */
        this.bj = bj;

        /**
         * SPOOK parameter
         * @property {number} a
         */
        this.a = 0.0;

        /**
         * SPOOK parameter
         * @property {number} b
         */
        this.b = 0.0;

        /**
         * SPOOK parameter
         * @property {number} eps
         */
        this.eps = 0.0;

        /**
         * @property {JacobianElement} jacobianElementA
         */
        this.jacobianElementA = new JacobianElement();

        /**
         * @property {JacobianElement} jacobianElementB
         */
        this.jacobianElementB = new JacobianElement();

        /**
         * @property {boolean} enabled
         * @default true
         */
        this.enabled = true;

        // Set typical spook params
        this.setSpookParams(1e7,4,1/60);
    }

    /**
     * Recalculates a,b,eps.
     * @method setSpookParams
     */
    setSpookParams(stiffness, relaxation, timeStep) {
        const d = relaxation;
        const k = stiffness;
        const h = timeStep;
        this.a = 4.0 / (h * (1 + 4 * d));
        this.b = (4.0 * d) / (1 + 4 * d);
        this.eps = 4.0 / (h * h * k * (1 + 4 * d));
    }

    /**
     * Computes the RHS of the SPOOK equation
     * @method computeB
     * @return {Number}
     */
    computeB(a, b, h) {
        const GW = this.computeGW();
        const Gq = this.computeGq();
        const GiMf = this.computeGiMf();
        return - Gq * a - GW * b - GiMf*h;
    }

    /**
     * Computes G*q, where q are the generalized body coordinates
     * @method computeGq
     * @return {Number}
     */
    computeGq() {
        const GA = this.jacobianElementA;
        const GB = this.jacobianElementB;
        const bi = this.bi;
        const bj = this.bj;
        const xi = bi.position;
        const xj = bj.position;
        return GA.spatial.dot(xi) + GB.spatial.dot(xj);
    }

    /**
     * Computes G*W, where W are the body velocities
     * @method computeGW
     * @return {Number}
     */
    computeGW() {
        const GA = this.jacobianElementA;
        const GB = this.jacobianElementB;
        const bi = this.bi;
        const bj = this.bj;
        const vi = bi.velocity;
        const vj = bj.velocity;
        const wi = bi.angularVelocity || zero;
        const wj = bj.angularVelocity || zero;
        return GA.multiplyVectors(vi,wi) + GB.multiplyVectors(vj,wj);
    }

    /**
     * Computes G*Wlambda, where W are the body velocities
     * @method computeGWlambda
     * @return {Number}
     */
    computeGWlambda() {
        const GA = this.jacobianElementA;
        const GB = this.jacobianElementB;
        const bi = this.bi;
        const bj = this.bj;
        const vi = bi.vlambda;
        const vj = bj.vlambda;
        const wi = bi.wlambda || zero;
        const wj = bj.wlambda || zero;
        return GA.multiplyVectors(vi,wi) + GB.multiplyVectors(vj,wj);
    }

    computeGiMf() {
        const GA = this.jacobianElementA;
        const GB = this.jacobianElementB;
        const bi = this.bi;
        const bj = this.bj;
        const fi = bi.force;
        const ti = bi.torque;
        const fj = bj.force;
        const tj = bj.torque;
        const invMassi = bi.invMassSolve;
        const invMassj = bj.invMassSolve;

        if(bi.invInertiaWorldSolve){ bi.invInertiaWorldSolve.vmult(ti,invIi_vmult_taui); }
        else { invIi_vmult_taui.set(0,0,0); }
        if(bj.invInertiaWorldSolve){ bj.invInertiaWorldSolve.vmult(tj,invIj_vmult_tauj); }
        else { invIj_vmult_tauj.set(0,0,0); }

        fi.mult(invMassi,iMfi);
        fj.mult(invMassj,iMfj);

        return GA.multiplyVectors(iMfi,invIi_vmult_taui) + GB.multiplyVectors(iMfj,invIj_vmult_tauj);
    }

    computeGiMGt() {
        const GA = this.jacobianElementA;
        const GB = this.jacobianElementB;
        const bi = this.bi;
        const bj = this.bj;
        const invMassi = bi.invMassSolve;
        const invMassj = bj.invMassSolve;
        const invIi = bi.invInertiaWorldSolve;
        const invIj = bj.invInertiaWorldSolve;
        let result = invMassi + invMassj;

        if(invIi){
            invIi.vmult(GA.rotational,tmp);
            result += tmp.dot(GA.rotational);
        }

        if(invIj){
            invIj.vmult(GB.rotational,tmp);
            result += tmp.dot(GB.rotational);
        }

        return  result;
    }

    /**
     * Add constraint velocity to the bodies.
     * @method addToWlambda
     * @param {Number} deltalambda
     */
    addToWlambda(deltalambda) {
        const GA = this.jacobianElementA;
        const GB = this.jacobianElementB;
        const bi = this.bi;
        const bj = this.bj;
        const temp = addToWlambda_temp;

        // Add to linear velocity
        // v_lambda += inv(M) * delta_lamba * G
        GA.spatial.mult(bi.invMassSolve * deltalambda,temp);
        bi.vlambda.vadd(temp, bi.vlambda);

        GB.spatial.mult(bj.invMassSolve * deltalambda,temp);
        bj.vlambda.vadd(temp, bj.vlambda);

        // Add to angular velocity
        if(bi.invInertiaWorldSolve){
            bi.invInertiaWorldSolve.vmult(GA.rotational,temp);
            temp.mult(deltalambda,temp);
            bi.wlambda.vadd(temp,bi.wlambda);
        }

        if(bj.invInertiaWorldSolve){
            bj.invInertiaWorldSolve.vmult(GB.rotational,temp);
            temp.mult(deltalambda,temp);
            bj.wlambda.vadd(temp,bj.wlambda);
        }
    }

    /**
     * Compute the denominator part of the SPOOK equation: C = G*inv(M)*G' + eps
     * @method computeInvC
     * @param  {Number} eps
     * @return {Number}
     */
    computeC() {
        return this.computeGiMGt() + this.eps;
    }
}

Equation.id = 0;

var zero = new Vec3();

/**
 * Computes G*inv(M)*f, where M is the mass matrix with diagonal blocks for each body, and f are the forces on the bodies.
 * @method computeGiMf
 * @return {Number}
 */
var iMfi = new Vec3();

var iMfj = new Vec3();
var invIi_vmult_taui = new Vec3();
var invIj_vmult_tauj = new Vec3();

/**
 * Computes G*inv(M)*G'
 * @method computeGiMGt
 * @return {Number}
 */
var tmp = new Vec3();
var addToWlambda_temp = new Vec3();
const addToWlambda_Gi = new Vec3();
const addToWlambda_Gj = new Vec3();
const addToWlambda_ri = new Vec3();
const addToWlambda_rj = new Vec3();
const addToWlambda_Mdiag = new Vec3();

export default Equation;
