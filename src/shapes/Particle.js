import Shape from './Shape';
import Vec3 from '../math/Vec3';

/**
 * Particle shape.
 * @class Particle
 * @constructor
 * @author schteppe
 * @extends Shape
 */
class Particle {
    constructor() {
        Shape.call(this);

        this.type = Shape.types.PARTICLE;
    }

    updateBoundingSphereRadius() {
        this.boundingSphereRadius = 0;
    }
}

Particle.prototype = new Shape();
Particle.prototype.constructor = Particle;

/**
 * @method calculateLocalInertia
 * @param  {Number} mass
 * @param  {Vec3} target
 * @return {Vec3}
 */
Particle.prototype.calculateLocalInertia = (mass, target) => {
    target = target || new Vec3();
    target.set(0, 0, 0);
    return target;
};

Particle.prototype.volume = () => 0;

Particle.prototype.calculateWorldAABB = (pos, quat, min, max) => {
    // Get each axis max
    min.copy(pos);
    max.copy(pos);
};

export default Particle;
