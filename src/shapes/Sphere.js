import Shape from './Shape';
import Vec3 from '../math/Vec3';

/**
 * Spherical shape
 * @class Sphere
 * @constructor
 * @extends Shape
 * @param {Number} radius The radius of the sphere, a non-negative number.
 * @author schteppe / http://github.com/schteppe
 */
class Sphere extends Shape {
    constructor(radius) {
        super();
        Shape.call(this);

        /**
         * @property {Number} radius
         */
        this.radius = radius!==undefined ? Number(radius) : 1.0;
        this.type = Shape.types.SPHERE;

        if(this.radius < 0){
            throw new Error('The sphere radius cannot be negative.');
        }

        this.updateBoundingSphereRadius();
    }

    calculateLocalInertia(mass, target=new Vec3()) {
        const I = 2.0*mass*this.radius*this.radius/5.0;
        target.x = I;
        target.y = I;
        target.z = I;
        return target;
    }

    volume() {
        return 4.0 * Math.PI * this.radius / 3.0;
    }

    updateBoundingSphereRadius() {
        this.boundingSphereRadius = this.radius;
    }

    calculateWorldAABB(pos, quat, min, max) {
        const r = this.radius;
        const axes = ['x','y','z'];

        for (const ax of axes) {
            min[ax] = pos[ax] - r;
            max[ax] = pos[ax] + r;
        }
    }
}

export default Sphere;
