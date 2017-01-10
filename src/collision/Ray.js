import Vec3 from '../math/Vec3';
import Quaternion from '../math/Quaternion';
import Transform from '../math/Transform';
import ConvexPolyhedron from '../shapes/ConvexPolyhedron';
import Box from '../shapes/Box';
import RaycastResult from '../collision/RaycastResult';
import Shape from '../shapes/Shape';
import AABB from '../collision/AABB';

/**
 * A line in 3D space that intersects bodies and return points.
 * @class Ray
 * @constructor
 * @param {Vec3} from
 * @param {Vec3} to
 */
class Ray {
    constructor(from, to) {
        /**
         * @property {Vec3} from
         */
        this.from = from ? from.clone() : new Vec3();

        /**
         * @property {Vec3} to
         */
        this.to = to ? to.clone() : new Vec3();

        /**
         * @private
         * @property {Vec3} _direction
         */
        this._direction = new Vec3();

        /**
         * The precision of the ray. Used when checking parallelity etc.
         * @property {Number} precision
         */
        this.precision = 0.0001;

        /**
         * Set to true if you want the Ray to take .collisionResponse flags into account on bodies and shapes.
         * @property {Boolean} checkCollisionResponse
         */
        this.checkCollisionResponse = true;

        /**
         * If set to true, the ray skips any hits with normal.dot(rayDirection) < 0.
         * @property {Boolean} skipBackfaces
         */
        this.skipBackfaces = false;

        /**
         * @property {number} collisionFilterMask
         * @default -1
         */
        this.collisionFilterMask = -1;

        /**
         * @property {number} collisionFilterGroup
         * @default -1
         */
        this.collisionFilterGroup = -1;

        /**
         * The intersection mode. Should be Ray.ANY, Ray.ALL or Ray.CLOSEST.
         * @property {number} mode
         */
        this.mode = Ray.ANY;

        /**
         * Current result object.
         * @property {RaycastResult} result
         */
        this.result = new RaycastResult();

        /**
         * Will be set to true during intersectWorld() if the ray hit anything.
         * @property {Boolean} hasHit
         */
        this.hasHit = false;

        /**
         * Current, user-provided result callback. Will be used if mode is Ray.ALL.
         * @property {Function} callback
         */
        this.callback = result => {};
    }

    /**
     * Do itersection against all bodies in the given World.
     * @method intersectWorld
     * @param  {World} world
     * @param  {object} options
     * @return {Boolean} True if the ray hit anything, otherwise false.
     */
    intersectWorld(world, options) {
        this.mode = options.mode || Ray.ANY;
        this.result = options.result || new RaycastResult();
        this.skipBackfaces = !!options.skipBackfaces;
        this.collisionFilterMask = typeof(options.collisionFilterMask) !== 'undefined' ? options.collisionFilterMask : -1;
        this.collisionFilterGroup = typeof(options.collisionFilterGroup) !== 'undefined' ? options.collisionFilterGroup : -1;
        if(options.from){
            this.from.copy(options.from);
        }
        if(options.to){
            this.to.copy(options.to);
        }
        this.callback = options.callback || (() => {});
        this.hasHit = false;

        this.result.reset();
        this._updateDirection();

        this.getAABB(tmpAABB);
        tmpArray.length = 0;
        world.broadphase.aabbQuery(world, tmpAABB, tmpArray);
        this.intersectBodies(tmpArray);

        return this.hasHit;
    }

    intersectBody(body, result) {
        if(result){
            this.result = result;
            this._updateDirection();
        }
        const checkCollisionResponse = this.checkCollisionResponse;

        if(checkCollisionResponse && !body.collisionResponse){
            return;
        }

        if((this.collisionFilterGroup & body.collisionFilterMask)===0 || (body.collisionFilterGroup & this.collisionFilterMask)===0){
            return;
        }

        const xi = intersectBody_xi;
        const qi = intersectBody_qi;

        for (let i = 0, N = body.shapes.length; i < N; i++) {
            const shape = body.shapes[i];

            if(checkCollisionResponse && !shape.collisionResponse){
                continue; // Skip
            }

            body.quaternion.mult(body.shapeOrientations[i], qi);
            body.quaternion.vmult(body.shapeOffsets[i], xi);
            xi.vadd(body.position, xi);

            this.intersectShape(
                shape,
                qi,
                xi,
                body
            );

            if(this.result._shouldStop){
                break;
            }
        }
    }

    /**
     * @method intersectBodies
     * @param {Array} bodies An array of Body objects.
     * @param {RaycastResult} [result] Deprecated
     */
    intersectBodies(bodies, result) {
        if(result){
            this.result = result;
            this._updateDirection();
        }

        for ( let i = 0, l = bodies.length; !this.result._shouldStop && i < l; i ++ ) {
            this.intersectBody(bodies[i]);
        }
    }

    /**
     * Updates the _direction vector.
     * @private
     * @method _updateDirection
     */
    _updateDirection() {
        this.to.vsub(this.from, this._direction);
        this._direction.normalize();
    }

    /**
     * @method intersectShape
     * @private
     * @param {Shape} shape
     * @param {Quaternion} quat
     * @param {Vec3} position
     * @param {Body} body
     */
    intersectShape(shape, quat, position, body) {
        const from = this.from;


        // Checking boundingSphere
        const distance = distanceFromIntersection(from, this._direction, position);
        if ( distance > shape.boundingSphereRadius ) {
            return;
        }

        const intersectMethod = this[shape.type];
        if(intersectMethod){
            intersectMethod.call(this, shape, quat, position, body);
        }
    }

    /**
     * @method intersectBox
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     */
    intersectBox(shape, quat, position, body) {
        return this.intersectConvex(shape.convexPolyhedronRepresentation, quat, position, body);
    }

    /**
     * @method intersectPlane
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     */
    intersectPlane(shape, quat, position, body) {
        const from = this.from;
        const to = this.to;
        const direction = this._direction;

        // Get plane normal
        const worldNormal = new Vec3(0, 0, 1);
        quat.vmult(worldNormal, worldNormal);

        const len = new Vec3();
        from.vsub(position, len);
        const planeToFrom = len.dot(worldNormal);
        to.vsub(position, len);
        const planeToTo = len.dot(worldNormal);

        if(planeToFrom * planeToTo > 0){
            // "from" and "to" are on the same side of the plane... bail out
            return;
        }

        if(from.distanceTo(to) < planeToFrom){
            return;
        }

        const n_dot_dir = worldNormal.dot(direction);

        if (Math.abs(n_dot_dir) < this.precision) {
            // No intersection
            return;
        }

        const planePointToFrom = new Vec3();
        const dir_scaled_with_t = new Vec3();
        const hitPointWorld = new Vec3();

        from.vsub(position, planePointToFrom);
        const t = -worldNormal.dot(planePointToFrom) / n_dot_dir;
        direction.scale(t, dir_scaled_with_t);
        from.vadd(dir_scaled_with_t, hitPointWorld);

        this.reportIntersection(worldNormal, hitPointWorld, shape, body, -1);
    }

    /**
     * Get the world AABB of the ray.
     * @method getAABB
     * @param  {AABB} aabb
     */
    getAABB(result) {
        const to = this.to;
        const from = this.from;
        result.lowerBound.x = Math.min(to.x, from.x);
        result.lowerBound.y = Math.min(to.y, from.y);
        result.lowerBound.z = Math.min(to.z, from.z);
        result.upperBound.x = Math.max(to.x, from.x);
        result.upperBound.y = Math.max(to.y, from.y);
        result.upperBound.z = Math.max(to.z, from.z);
    }

    /**
     * @method intersectHeightfield
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     */
    intersectHeightfield(shape, quat, position, body) {
        const data = shape.data;
        const w = shape.elementSize;
        const worldPillarOffset = new Vec3();

        // Convert the ray to local heightfield coordinates
        const localRay = new Ray(this.from, this.to);
        Transform.pointToLocalFrame(position, quat, localRay.from, localRay.from);
        Transform.pointToLocalFrame(position, quat, localRay.to, localRay.to);

        // Get the index of the data points to test against
        const index = [];
        let iMinX = null;
        let iMinY = null;
        let iMaxX = null;
        let iMaxY = null;

        let inside = shape.getIndexOfPosition(localRay.from.x, localRay.from.y, index, false);
        if(inside){
            iMinX = index[0];
            iMinY = index[1];
            iMaxX = index[0];
            iMaxY = index[1];
        }
        inside = shape.getIndexOfPosition(localRay.to.x, localRay.to.y, index, false);
        if(inside){
            if (iMinX === null || index[0] < iMinX) { iMinX = index[0]; }
            if (iMaxX === null || index[0] > iMaxX) { iMaxX = index[0]; }
            if (iMinY === null || index[1] < iMinY) { iMinY = index[1]; }
            if (iMaxY === null || index[1] > iMaxY) { iMaxY = index[1]; }
        }

        if(iMinX === null){
            return;
        }

        const minMax = [];
        shape.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
        const min = minMax[0];
        const max = minMax[1];

        // // Bail out if the ray can't touch the bounding box
        // // TODO
        // var aabb = new AABB();
        // this.getAABB(aabb);
        // if(aabb.intersects()){
        //     return;
        // }

        for(let i = iMinX; i <= iMaxX; i++){
            for(let j = iMinY; j <= iMaxY; j++){

                if(this.result._shouldStop){
                    return;
                }

                // Lower triangle
                shape.getConvexTrianglePillar(i, j, false);
                Transform.pointToWorldFrame(position, quat, shape.pillarOffset, worldPillarOffset);
                this.intersectConvex(shape.pillarConvex, quat, worldPillarOffset, body, intersectConvexOptions);

                if(this.result._shouldStop){
                    return;
                }

                // Upper triangle
                shape.getConvexTrianglePillar(i, j, true);
                Transform.pointToWorldFrame(position, quat, shape.pillarOffset, worldPillarOffset);
                this.intersectConvex(shape.pillarConvex, quat, worldPillarOffset, body, intersectConvexOptions);
            }
        }
    }

    /**
     * @method intersectSphere
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     */
    intersectSphere(shape, quat, position, body) {
        const from = this.from;
        const to = this.to;
        const r = shape.radius;

        const a = Math.pow(to.x - from.x, 2) + Math.pw(to.y - from.y, 2) + Math.pow(to.z - from.z, 2);
        const b = 2 * ((to.x - from.x) * (from.x - position.x) + (to.y - from.y) * (from.y - position.y) + (to.z - from.z) * (from.z - position.z));
        const c = Math.pow(from.x - position.x, 2) + Math.pow(from.y - position.y, 2) + Math.pow(from.z - position.z, 2) - Math.pow(r, 2);

        const delta = Math.pow(b, 2) - 4 * a * c;

        const intersectionPoint = Ray_intersectSphere_intersectionPoint;
        const normal = Ray_intersectSphere_normal;

        if(delta < 0){
            // No intersection
            return;

        } else if(delta === 0){
            // single intersection point
            from.lerp(to, delta, intersectionPoint);

            intersectionPoint.vsub(position, normal);
            normal.normalize();

            this.reportIntersection(normal, intersectionPoint, shape, body, -1);

        } else {
            const d1 = (- b - Math.sqrt(delta)) / (2 * a);
            const d2 = (- b + Math.sqrt(delta)) / (2 * a);

            if(d1 >= 0 && d1 <= 1){
                from.lerp(to, d1, intersectionPoint);
                intersectionPoint.vsub(position, normal);
                normal.normalize();
                this.reportIntersection(normal, intersectionPoint, shape, body, -1);
            }

            if(this.result._shouldStop){
                return;
            }

            if(d2 >= 0 && d2 <= 1){
                from.lerp(to, d2, intersectionPoint);
                intersectionPoint.vsub(position, normal);
                normal.normalize();
                this.reportIntersection(normal, intersectionPoint, shape, body, -1);
            }
        }
    }

    /**
     * @method intersectConvex
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     * @param {object} [options]
     * @param {array} [options.faceList]
     */
    intersectConvex(shape, quat, position, body, options) {
        const minDistNormal = intersectConvex_minDistNormal;
        const normal = intersectConvex_normal;
        const vector = intersectConvex_vector;
        const minDistIntersect = intersectConvex_minDistIntersect;
        const faceList = (options && options.faceList) || null;

        // Checking faces
        const faces = shape.faces;

        const vertices = shape.vertices;
        const normals = shape.faceNormals;
        const direction = this._direction;

        const from = this.from;
        const to = this.to;
        const fromToDistance = from.distanceTo(to);

        const minDist = -1;
        const Nfaces = faceList ? faceList.length : faces.length;
        const result = this.result;

        for (let j = 0; !result._shouldStop && j < Nfaces; j++) {
            const fi = faceList ? faceList[j] : j;

            const face = faces[fi];
            const faceNormal = normals[fi];
            const q = quat;
            const x = position;

            // determine if ray intersects the plane of the face
            // note: this works regardless of the direction of the face normal

            // Get plane point in world coordinates...
            vector.copy(vertices[face[0]]);
            q.vmult(vector,vector);
            vector.vadd(x,vector);

            // ...but make it relative to the ray from. We'll fix this later.
            vector.vsub(from,vector);

            // Get plane normal
            q.vmult(faceNormal,normal);

            // If this dot product is negative, we have something interesting
            const dot = direction.dot(normal);

            // Bail out if ray and plane are parallel
            if ( Math.abs( dot ) < this.precision ){
                continue;
            }

            // calc distance to plane
            const scalar = normal.dot(vector) / dot;

            // if negative distance, then plane is behind ray
            if (scalar < 0){
                continue;
            }

            // if (dot < 0) {

            // Intersection point is from + direction * scalar
            direction.mult(scalar,intersectPoint);
            intersectPoint.vadd(from,intersectPoint);

            // a is the point we compare points b and c with.
            a.copy(vertices[face[0]]);
            q.vmult(a,a);
            x.vadd(a,a);

            for(let i = 1; !result._shouldStop && i < face.length - 1; i++){
                // Transform 3 vertices to world coords
                b.copy(vertices[face[i]]);
                c.copy(vertices[face[i+1]]);
                q.vmult(b,b);
                q.vmult(c,c);
                x.vadd(b,b);
                x.vadd(c,c);

                const distance = intersectPoint.distanceTo(from);

                if(!(pointInTriangle(intersectPoint, a, b, c) || pointInTriangle(intersectPoint, b, a, c)) || distance > fromToDistance){
                    continue;
                }

                this.reportIntersection(normal, intersectPoint, shape, body, fi);
            }
            // }
        }
    }

    /**
     * @method intersectTrimesh
     * @private
     * @param  {Shape} shape
     * @param  {Quaternion} quat
     * @param  {Vec3} position
     * @param  {Body} body
     * @param {object} [options]
     * @todo Optimize by transforming the world to local space first.
     * @todo Use Octree lookup
     */
    intersectTrimesh(mesh, quat, position, body, options) {
        const normal = intersectTrimesh_normal;
        const triangles = intersectTrimesh_triangles;
        const treeTransform = intersectTrimesh_treeTransform;
        const minDistNormal = intersectConvex_minDistNormal;
        const vector = intersectConvex_vector;
        const minDistIntersect = intersectConvex_minDistIntersect;
        const localAABB = intersectTrimesh_localAABB;
        const localDirection = intersectTrimesh_localDirection;
        const localFrom = intersectTrimesh_localFrom;
        const localTo = intersectTrimesh_localTo;
        const worldIntersectPoint = intersectTrimesh_worldIntersectPoint;
        const worldNormal = intersectTrimesh_worldNormal;
        const faceList = (options && options.faceList) || null;

        // Checking faces
        const indices = mesh.indices;

        const vertices = mesh.vertices;
        const normals = mesh.faceNormals;

        const from = this.from;
        const to = this.to;
        const direction = this._direction;

        const minDist = -1;
        treeTransform.position.copy(position);
        treeTransform.quaternion.copy(quat);

        // Transform ray to local space!
        Transform.vectorToLocalFrame(position, quat, direction, localDirection);
        //body.vectorToLocalFrame(direction, localDirection);
        Transform.pointToLocalFrame(position, quat, from, localFrom);
        //body.pointToLocalFrame(from, localFrom);
        Transform.pointToLocalFrame(position, quat, to, localTo);
        //body.pointToLocalFrame(to, localTo);
        const fromToDistanceSquared = localFrom.distanceSquared(localTo);

        mesh.tree.rayQuery(this, treeTransform, triangles);

        for (let i = 0, N = triangles.length; !this.result._shouldStop && i !== N; i++) {
            const trianglesIndex = triangles[i];

            mesh.getNormal(trianglesIndex, normal);

            // determine if ray intersects the plane of the face
            // note: this works regardless of the direction of the face normal

            // Get plane point in world coordinates...
            mesh.getVertex(indices[trianglesIndex * 3], a);

            // ...but make it relative to the ray from. We'll fix this later.
            a.vsub(localFrom,vector);

            // Get plane normal
            // quat.vmult(normal, normal);

            // If this dot product is negative, we have something interesting
            const dot = localDirection.dot(normal);

            // Bail out if ray and plane are parallel
            // if (Math.abs( dot ) < this.precision){
            //     continue;
            // }

            // calc distance to plane
            const scalar = normal.dot(vector) / dot;

            // if negative distance, then plane is behind ray
            if (scalar < 0){
                continue;
            }

            // Intersection point is from + direction * scalar
            localDirection.scale(scalar,intersectPoint);
            intersectPoint.vadd(localFrom,intersectPoint);

            // Get triangle vertices
            mesh.getVertex(indices[trianglesIndex * 3 + 1], b);
            mesh.getVertex(indices[trianglesIndex * 3 + 2], c);

            const squaredDistance = intersectPoint.distanceSquared(localFrom);

            if(!(pointInTriangle(intersectPoint, b, a, c) || pointInTriangle(intersectPoint, a, b, c)) || squaredDistance > fromToDistanceSquared){
                continue;
            }

            // transform intersectpoint and normal to world
            Transform.vectorToWorldFrame(quat, normal, worldNormal);
            //body.vectorToWorldFrame(normal, worldNormal);
            Transform.pointToWorldFrame(position, quat, intersectPoint, worldIntersectPoint);
            //body.pointToWorldFrame(intersectPoint, worldIntersectPoint);
            this.reportIntersection(worldNormal, worldIntersectPoint, mesh, body, trianglesIndex);
        }
        triangles.length = 0;
    }

    /**
     * @method reportIntersection
     * @private
     * @param  {Vec3} normal
     * @param  {Vec3} hitPointWorld
     * @param  {Shape} shape
     * @param  {Body} body
     * @return {boolean} True if the intersections should continue
     */
    reportIntersection(normal, hitPointWorld, shape, body, hitFaceIndex) {
        const from = this.from;
        const to = this.to;
        const distance = from.distanceTo(hitPointWorld);
        const result = this.result;

        // Skip back faces?
        if(this.skipBackfaces && normal.dot(this._direction) > 0){
            return;
        }

        result.hitFaceIndex = typeof(hitFaceIndex) !== 'undefined' ? hitFaceIndex : -1;

        switch(this.mode){
        case Ray.ALL:
            this.hasHit = true;
            result.set(
                from,
                to,
                normal,
                hitPointWorld,
                shape,
                body,
                distance
            );
            result.hasHit = true;
            this.callback(result);
            break;

        case Ray.CLOSEST:

            // Store if closer than current closest
            if(distance < result.distance || !result.hasHit){
                this.hasHit = true;
                result.hasHit = true;
                result.set(
                    from,
                    to,
                    normal,
                    hitPointWorld,
                    shape,
                    body,
                    distance
                );
            }
            break;

        case Ray.ANY:

            // Report and stop.
            this.hasHit = true;
            result.hasHit = true;
            result.set(
                from,
                to,
                normal,
                hitPointWorld,
                shape,
                body,
                distance
            );
            result._shouldStop = true;
            break;
        }
    }
}

Ray.prototype.constructor = Ray;

Ray.CLOSEST = 1;
Ray.ANY = 2;
Ray.ALL = 4;

var tmpAABB = new AABB();
var tmpArray = [];

const v1 = new Vec3();
const v2 = new Vec3();

/*
 * As per "Barycentric Technique" as named here http://www.blackpawn.com/texts/pointinpoly/default.html But without the division
 */
Ray.pointInTriangle = pointInTriangle;
function pointInTriangle(p, a, b, c) {
    c.vsub(a,v0);
    b.vsub(a,v1);
    p.vsub(a,v2);

    const dot00 = v0.dot( v0 );
    const dot01 = v0.dot( v1 );
    const dot02 = v0.dot( v2 );
    const dot11 = v1.dot( v1 );
    const dot12 = v1.dot( v2 );

    let u;
    let v;

    return  ( (u = dot11 * dot02 - dot01 * dot12) >= 0 ) &&
            ( (v = dot00 * dot12 - dot01 * dot02) >= 0 ) &&
            ( u + v < ( dot00 * dot11 - dot01 * dot01 ) );
}

/**
 * Shoot a ray at a body, get back information about the hit.
 * @method intersectBody
 * @private
 * @param {Body} body
 * @param {RaycastResult} [result] Deprecated - set the result property of the Ray instead.
 */
var intersectBody_xi = new Vec3();
var intersectBody_qi = new Quaternion();

const vector = new Vec3();
const normal = new Vec3();
var intersectPoint = new Vec3();

var a = new Vec3();
var b = new Vec3();
var c = new Vec3();
const d = new Vec3();

const tmpRaycastResult = new RaycastResult();

Ray.prototype[Shape.types.BOX] = Ray.prototype.intersectBox;

Ray.prototype[Shape.types.PLANE] = Ray.prototype.intersectPlane;

var intersectConvexOptions = {
    faceList: [0]
};

Ray.prototype[Shape.types.HEIGHTFIELD] = Ray.prototype.intersectHeightfield;

var Ray_intersectSphere_intersectionPoint = new Vec3();
var Ray_intersectSphere_normal = new Vec3();

Ray.prototype[Shape.types.SPHERE] = Ray.prototype.intersectSphere;


var intersectConvex_normal = new Vec3();
var intersectConvex_minDistNormal = new Vec3();
var intersectConvex_minDistIntersect = new Vec3();
var intersectConvex_vector = new Vec3();

Ray.prototype[Shape.types.CONVEXPOLYHEDRON] = Ray.prototype.intersectConvex;

var intersectTrimesh_normal = new Vec3();
var intersectTrimesh_localDirection = new Vec3();
var intersectTrimesh_localFrom = new Vec3();
var intersectTrimesh_localTo = new Vec3();
var intersectTrimesh_worldNormal = new Vec3();
var intersectTrimesh_worldIntersectPoint = new Vec3();
var intersectTrimesh_localAABB = new AABB();
var intersectTrimesh_triangles = [];
var intersectTrimesh_treeTransform = new Transform();

Ray.prototype[Shape.types.TRIMESH] = Ray.prototype.intersectTrimesh;


var v0 = new Vec3();
const intersect = new Vec3();
function distanceFromIntersection(from, direction, position) {

    // v0 is vector from from to position
    position.vsub(from,v0);
    const dot = v0.dot(direction);

    // intersect = direction*dot + from
    direction.mult(dot,intersect);
    intersect.vadd(from,intersect);

    const distance = position.distanceTo(intersect);

    return distance;
}

export default Ray;
