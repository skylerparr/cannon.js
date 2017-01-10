import Vec3 from './Vec3';

/**
 * A Quaternion describes a rotation in 3D space. The Quaternion is mathematically defined as Q = x*i + y*j + z*k + w, where (i,j,k) are imaginary basis vectors. (x,y,z) can be seen as a vector related to the axis of rotation, while the real multiplier, w, is related to the amount of rotation.
 * @class Quaternion
 * @constructor
 * @param {Number} x Multiplier of the imaginary basis vector i.
 * @param {Number} y Multiplier of the imaginary basis vector j.
 * @param {Number} z Multiplier of the imaginary basis vector k.
 * @param {Number} w Multiplier of the real part.
 * @see http://en.wikipedia.org/wiki/Quaternion
 */
class Quaternion {
    constructor(x, y, z, w) {
        /**
         * @property {Number} x
         */
        this.x = x!==undefined ? x : 0;

        /**
         * @property {Number} y
         */
        this.y = y!==undefined ? y : 0;

        /**
         * @property {Number} z
         */
        this.z = z!==undefined ? z : 0;

        /**
         * The multiplier of the real quaternion basis vector.
         * @property {Number} w
         */
        this.w = w!==undefined ? w : 1;
    }

    /**
     * Set the value of the quaternion.
     * @method set
     * @param {Number} x
     * @param {Number} y
     * @param {Number} z
     * @param {Number} w
     */
    set(x, y, z, w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * Convert to a readable format
     * @method toString
     * @return string
     */
    toString() {
        return `${this.x},${this.y},${this.z},${this.w}`;
    }

    /**
     * Convert to an Array
     * @method toArray
     * @return Array
     */
    toArray() {
        return [this.x, this.y, this.z, this.w];
    }

    /**
     * Set the quaternion components given an axis and an angle.
     * @method setFromAxisAngle
     * @param {Vec3} axis
     * @param {Number} angle in radians
     */
    setFromAxisAngle(axis, angle) {
        const s = Math.sin(angle*0.5);
        this.x = axis.x * s;
        this.y = axis.y * s;
        this.z = axis.z * s;
        this.w = Math.cos(angle*0.5);
    }

    /**
     * Converts the quaternion to axis/angle representation.
     * @method toAxisAngle
     * @param {Vec3} targetAxis Optional. A vector object to reuse for storing the axis.
     * @return Array An array, first elemnt is the axis and the second is the angle in radians.
     */
    toAxisAngle(targetAxis=new Vec3()) {
        this.normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
        const angle = 2 * Math.acos(this.w);
        const s = Math.sqrt(1-this.w*this.w); // assuming quaternion normalised then w is less than 1, so term always positive.
        if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
            // if s close to zero then direction of axis not important
            targetAxis.x = this.x; // if it is important that axis is normalised then replace with x=1; y=z=0;
            targetAxis.y = this.y;
            targetAxis.z = this.z;
        } else {
            targetAxis.x = this.x / s; // normalise axis
            targetAxis.y = this.y / s;
            targetAxis.z = this.z / s;
        }
        return [targetAxis,angle];
    }

    /**
     * Set the quaternion value given two vectors. The resulting rotation will be the needed rotation to rotate u to v.
     * @method setFromVectors
     * @param {Vec3} u
     * @param {Vec3} v
     */
    setFromVectors(u, v) {
        if(u.isAntiparallelTo(v)){
            const t1 = sfv_t1;
            const t2 = sfv_t2;

            u.tangents(t1,t2);
            this.setFromAxisAngle(t1,Math.PI);
        } else {
            const a = u.cross(v);
            this.x = a.x;
            this.y = a.y;
            this.z = a.z;
            this.w = Math.sqrt(Math.pow(u.norm(), 2) * (Math.pow(v.norm(), 2))) + u.dot(v);
            this.normalize();
        }
    }

    mult(q, target=new Quaternion()) {
        const w = this.w;
        const va = Quaternion_mult_va;
        const vb = Quaternion_mult_vb;
        const vaxvb = Quaternion_mult_vaxvb;

        va.set(this.x,this.y,this.z);
        vb.set(q.x,q.y,q.z);
        target.w = w*q.w - va.dot(vb);
        va.cross(vb,vaxvb);

        target.x = w * vb.x + q.w*va.x + vaxvb.x;
        target.y = w * vb.y + q.w*va.y + vaxvb.y;
        target.z = w * vb.z + q.w*va.z + vaxvb.z;

        return target;
    }

    /**
     * Get the inverse quaternion rotation.
     * @method inverse
     * @param {Quaternion} target
     * @return {Quaternion}
     */
    inverse(target) {
        const x = this.x;
        const y = this.y;
        const z = this.z;
        const w = this.w;
        target = target || new Quaternion();

        this.conjugate(target);
        const inorm2 = 1/(x*x + y*y + z*z + w*w);
        target.x *= inorm2;
        target.y *= inorm2;
        target.z *= inorm2;
        target.w *= inorm2;

        return target;
    }

    /**
     * Get the quaternion conjugate
     * @method conjugate
     * @param {Quaternion} target
     * @return {Quaternion}
     */
    conjugate(target=new Quaternion()) {
        target.x = -this.x;
        target.y = -this.y;
        target.z = -this.z;
        target.w = this.w;

        return target;
    }

    /**
     * Normalize the quaternion. Note that this changes the values of the quaternion.
     * @method normalize
     */
    normalize() {
        let l = Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w);
        if ( l === 0 ) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        } else {
            l = 1 / l;
            this.x *= l;
            this.y *= l;
            this.z *= l;
            this.w *= l;
        }
    }

    /**
     * Approximation of quaternion normalization. Works best when quat is already almost-normalized.
     * @method normalizeFast
     * @see http://jsperf.com/fast-quaternion-normalization
     * @author unphased, https://github.com/unphased
     */
    normalizeFast() {
        const f = (3.0-(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w))/2.0;
        if ( f === 0 ) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        } else {
            this.x *= f;
            this.y *= f;
            this.z *= f;
            this.w *= f;
        }
    }

    /**
     * Multiply the quaternion by a vector
     * @method vmult
     * @param {Vec3} v
     * @param {Vec3} target Optional
     * @return {Vec3}
     */
    vmult(v, target=new Vec3()) {
        const x = v.x;
        const y = v.y;
        const z = v.z;
        const qx = this.x;
        const qy = this.y;
        const qz = this.z;
        const qw = this.w;

        // q*v
        const ix =  qw * x + qy * z - qz * y;

        const iy =  qw * y + qz * x - qx * z;
        const iz =  qw * z + qx * y - qy * x;
        const iw = -qx * x - qy * y - qz * z;

        target.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        target.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        target.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;

        return target;
    }

    /**
     * Copies value of source to this quaternion.
     * @method copy
     * @param {Quaternion} source
     * @return {Quaternion} this
     */
    copy(source) {
        this.x = source.x;
        this.y = source.y;
        this.z = source.z;
        this.w = source.w;
        return this;
    }

    /**
     * Convert the quaternion to euler angle representation. Order: YZX, as this page describes: http://www.euclideanspace.com/maths/standards/index.htm
     * @method toEuler
     * @param {Vec3} target
     * @param string order Three-character string e.g. "YZX", which also is default.
     */
    toEuler(target, order="YZX") {
        let heading;
        let attitude;
        let bank;
        const x = this.x;
        const y = this.y;
        const z = this.z;
        const w = this.w;

        switch(order){
        case "YZX":
            const test = x*y + z*w;
            if (test > 0.499) { // singularity at north pole
                heading = 2 * Math.atan2(x,w);
                attitude = Math.PI/2;
                bank = 0;
            }
            if (test < -0.499) { // singularity at south pole
                heading = -2 * Math.atan2(x,w);
                attitude = - Math.PI/2;
                bank = 0;
            }
            if(isNaN(heading)){
                const sqx = x*x;
                const sqy = y*y;
                const sqz = z*z;
                heading = Math.atan2(2*y*w - 2*x*z , 1 - 2*sqy - 2*sqz); // Heading
                attitude = Math.asin(2*test); // attitude
                bank = Math.atan2(2*x*w - 2*y*z , 1 - 2*sqx - 2*sqz); // bank
            }
            break;
        default:
            throw new Error(`Euler order ${order} not supported yet.`);
        }

        target.y = heading;
        target.z = attitude;
        target.x = bank;
    }

    /**
     * See http://www.mathworks.com/matlabcentral/fileexchange/20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors/content/SpinCalc.m
     * @method setFromEuler
     * @param {Number} x
     * @param {Number} y
     * @param {Number} z
     * @param {String} order The order to apply angles: 'XYZ' or 'YXZ' or any other combination
     */
    setFromEuler(x, y, z, order="XYZ") {
        const c1 = Math.cos( x / 2 );
        const c2 = Math.cos( y / 2 );
        const c3 = Math.cos( z / 2 );
        const s1 = Math.sin( x / 2 );
        const s2 = Math.sin( y / 2 );
        const s3 = Math.sin( z / 2 );

        if ( order === 'XYZ' ) {

            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;

        } else if ( order === 'YXZ' ) {

            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;

        } else if ( order === 'ZXY' ) {

            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;

        } else if ( order === 'ZYX' ) {

            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;

        } else if ( order === 'YZX' ) {

            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;

        } else if ( order === 'XZY' ) {

            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;

        }

        return this;
    }

    clone() {
        return new Quaternion(this.x, this.y, this.z, this.w);
    }
}

var sfv_t1 = new Vec3();
var sfv_t2 = new Vec3();

/**
 * Quaternion multiplication
 * @method mult
 * @param {Quaternion} q
 * @param {Quaternion} target Optional.
 * @return {Quaternion}
 */
var Quaternion_mult_va = new Vec3();
var Quaternion_mult_vb = new Vec3();
var Quaternion_mult_vaxvb = new Vec3();

export default Quaternion;
