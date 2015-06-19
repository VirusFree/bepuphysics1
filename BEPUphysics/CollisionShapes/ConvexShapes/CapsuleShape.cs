using System;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;

using BEPUutilities;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{
    ///<summary>
    /// Sphere-expanded line segment.  Another way of looking at it is a cylinder with half-spheres on each end.
    ///</summary>
    public class CapsuleShape : ConvexShape
    {
        Axis _Axis = Axis.Y;
        public Axis Axis { get { return _Axis; } }

        //uncomment when/if needed
        //private Vector3 posGrowVector;
        //private Vector3 negGrowVector;


        float halfLength;
        ///<summary>
        /// Gets or sets the length of the capsule's inner line segment.
        ///</summary>
        public float Length
        {
            get
            {
                return halfLength * 2;
            }
            set
            {
                halfLength = value * 0.5f;
                OnShapeChanged();
            }
        }

        //This is a convenience method.  People expect to see a 'radius' of some kind.
        ///<summary>
        /// Gets or sets the radius of the capsule.
        ///</summary>
        public float Radius { get { return collisionMargin; } set { CollisionMargin = value; } }


        ///<summary>
        /// Constructs a new capsule shape.
        ///</summary>
        ///<param name="length">Length of the capsule's inner line segment.</param>
        ///<param name="radius">Radius to expand the line segment width.</param>
        public CapsuleShape(float length, float radius, Axis Axis = Axis.Y)
        {
            this._Axis = Axis;
            halfLength = length * 0.5f;
            //ChooseGrowVector();
            UpdateConvexShapeInfo(ComputeDescription(length, radius, Axis));
        }

        ///<summary>
        /// Constructs a new capsule shape from cached information.
        ///</summary>
        ///<param name="length">Length of the capsule's inner line segment.</param>
        /// <param name="description">Cached information about the shape. Assumed to be correct; no extra processing or validation is performed.</param>
        public CapsuleShape(float length, ConvexShapeDescription description, Axis Axis = Axis.Y)
        {
            this._Axis = Axis;
            halfLength = length * 0.5f;
            //ChooseGrowVector();
            UpdateConvexShapeInfo(description);
        }



        protected override void OnShapeChanged()
        {
            UpdateConvexShapeInfo(ComputeDescription(halfLength * 2, Radius, _Axis));
            base.OnShapeChanged();
        }


        /// <summary>
        /// Computes a convex shape description for a CapsuleShape.
        /// </summary>
        ///<param name="length">Length of the capsule's inner line segment.</param>
        ///<param name="radius">Radius to expand the line segment width.</param>
        /// <returns>Description required to define a convex shape.</returns>
        public static ConvexShapeDescription ComputeDescription(float length, float radius, Axis axis)
        {
            ConvexShapeDescription description;
            description.EntityShapeVolume.Volume = (float)(Math.PI * radius * radius * length + 1.333333 * Math.PI * radius * radius * radius);

            description.EntityShapeVolume.VolumeDistribution = new Matrix3x3();
            float effectiveLength = length + radius / 2; //This is a cylindrical inertia tensor. Approximate.
            float diagValue = (.0833333333f * effectiveLength * effectiveLength + .25f * radius * radius);
            if (axis == Axis.X)
            {
                description.EntityShapeVolume.VolumeDistribution.M11 = .5f * radius * radius;
                description.EntityShapeVolume.VolumeDistribution.M22 = diagValue;
                description.EntityShapeVolume.VolumeDistribution.M33 = diagValue;
            }
            else if (axis == Axis.Y)
            {
                description.EntityShapeVolume.VolumeDistribution.M11 = diagValue;
                description.EntityShapeVolume.VolumeDistribution.M22 = .5f * radius * radius;
                description.EntityShapeVolume.VolumeDistribution.M33 = diagValue;
            }
            else if (axis == Axis.Z)
            {
                description.EntityShapeVolume.VolumeDistribution.M11 = diagValue;
                description.EntityShapeVolume.VolumeDistribution.M22 = diagValue;
                description.EntityShapeVolume.VolumeDistribution.M33 = .5f * radius * radius;
            }

            description.MaximumRadius = length * 0.5f + radius;
            description.MinimumRadius = radius;

            description.CollisionMargin = radius;
            return description;
        }

        public override void GetBoundingBox(ref RigidTransform shapeTransform, out BoundingBox boundingBox)
        {
#if !WINDOWS
            boundingBox = new BoundingBox();
#endif
            Vector3 upExtreme;
            if (_Axis == Axis.X)
                Quaternion.TransformX(halfLength, ref shapeTransform.Orientation, out upExtreme);
            else if (_Axis == Axis.Y)
                Quaternion.TransformY(halfLength, ref shapeTransform.Orientation, out upExtreme);
            else if (_Axis == Axis.Z)
                Quaternion.TransformZ(halfLength, ref shapeTransform.Orientation, out upExtreme);
            else
                upExtreme = Vector3.Zero;

            if (upExtreme.X > 0)
            {
                boundingBox.Max.X = upExtreme.X + collisionMargin;
                boundingBox.Min.X = -upExtreme.X - collisionMargin;
            }
            else
            {
                boundingBox.Max.X = -upExtreme.X + collisionMargin;
                boundingBox.Min.X = upExtreme.X - collisionMargin;
            }

            if (upExtreme.Y > 0)
            {
                boundingBox.Max.Y = upExtreme.Y + collisionMargin;
                boundingBox.Min.Y = -upExtreme.Y - collisionMargin;
            }
            else
            {
                boundingBox.Max.Y = -upExtreme.Y + collisionMargin;
                boundingBox.Min.Y = upExtreme.Y - collisionMargin;
            }

            if (upExtreme.Z > 0)
            {
                boundingBox.Max.Z = upExtreme.Z + collisionMargin;
                boundingBox.Min.Z = -upExtreme.Z - collisionMargin;
            }
            else
            {
                boundingBox.Max.Z = -upExtreme.Z + collisionMargin;
                boundingBox.Min.Z = upExtreme.Z - collisionMargin;
            }

            Vector3.Add(ref shapeTransform.Position, ref boundingBox.Min, out boundingBox.Min);
            Vector3.Add(ref shapeTransform.Position, ref boundingBox.Max, out boundingBox.Max);
        }


        ///<summary>
        /// Gets the extreme point of the shape in local space in a given direction.
        ///</summary>
        ///<param name="direction">Direction to find the extreme point in.</param>
        ///<param name="extremePoint">Extreme point on the shape.</param>
        public override void GetLocalExtremePointWithoutMargin(ref Vector3 direction, out Vector3 extremePoint)
        {
            if (_Axis == Axis.X)
            {
                if (direction.X != 0)
                    extremePoint = new Vector3(Math.Sign(direction.X) * halfLength, 0, 0);
                else
                    extremePoint = Toolbox.ZeroVector;
            }
            else if (_Axis == Axis.Y)
            {
                if (direction.Y != 0)
                    extremePoint = new Vector3(0, Math.Sign(direction.Y) * halfLength, 0);
                else
                    extremePoint = Toolbox.ZeroVector;
            }
            else if (_Axis == Axis.Z)
            {
                if (direction.Z != 0)
                    extremePoint = new Vector3(0, 0, Math.Sign(direction.Z) * halfLength);
                else
                    extremePoint = Toolbox.ZeroVector;
            }
            else
                extremePoint = Toolbox.ZeroVector;
        }




        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new ConvexCollidable<CapsuleShape>(this);
        }

        /// <summary>
        /// Gets the intersection between the convex shape and the ray.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="transform">Transform of the convex shape.</param>
        /// <param name="maximumLength">Maximum distance to travel in units of the ray direction's length.</param>
        /// <param name="hit">Ray hit data, if any.</param>
        /// <returns>Whether or not the ray hit the target.</returns>
        public override bool RayTest(ref Ray ray, ref RigidTransform transform, float maximumLength, out RayHit hit)
        {
            //Put the ray into local space.
            Quaternion conjugate;
            Quaternion.Conjugate(ref transform.Orientation, out conjugate);
            Ray localRay;
            Vector3.Subtract(ref ray.Position, ref transform.Position, out localRay.Position);
            Quaternion.Transform(ref localRay.Position, ref conjugate, out localRay.Position);
            Quaternion.Transform(ref ray.Direction, ref conjugate, out localRay.Direction);

            //get growth elements
            float posGrow; float posSide1; float posSide2;
            float dirGrow; float dirSide1; float dirSide2;
            GetGrowElements(ref localRay.Position, out posGrow, out posSide1, out posSide2);
            GetGrowElements(ref localRay.Direction, out dirGrow, out dirSide1, out dirSide2);
            
            //Check for containment in the cylindrical portion of the capsule.
            if (posGrow >= -halfLength && posGrow <= halfLength && posSide1 * posSide1 + posSide2 * posSide2 <= collisionMargin * collisionMargin)
            {
                //It's inside!
                hit.T = 0;
                hit.Location = localRay.Position;
                CreateHitNormal(posSide1, posSide2, out hit.Normal);
                float normalLengthSquared = hit.Normal.LengthSquared();
                if (normalLengthSquared > 1e-9f)
                    Vector3.Divide(ref hit.Normal, (float)Math.Sqrt(normalLengthSquared), out hit.Normal);
                else
                    hit.Normal = default(Vector3);
                //Pull the hit into world space.
                Quaternion.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref hit.Location, ref transform, out hit.Location);
                return true;
            }

            //Project the ray direction onto the plane where the cylinder is a circle.
            //The projected ray is then tested against the circle to compute the time of impact.
            //That time of impact is used to compute the 3d hit location.
            Vector2 planeDirection = new Vector2(dirSide1, dirSide2);
            float planeDirectionLengthSquared = planeDirection.LengthSquared();

            if (planeDirectionLengthSquared < Toolbox.Epsilon)
            {
                //The ray is nearly parallel with the axis.
                //Skip the cylinder-sides test.  We're either inside the cylinder and won't hit the sides, or we're outside
                //and won't hit the sides.  
                if (posGrow > halfLength)
                    goto upperSphereTest;
                if (posGrow < -halfLength)
                    goto lowerSphereTest;


                hit = default(RayHit);
                return false;

            }
            Vector2 planeOrigin = new Vector2(posSide1, posSide2);
            float dot;
            Vector2.Dot(ref planeDirection, ref planeOrigin, out dot);
            float closestToCenterT = -dot / planeDirectionLengthSquared;

            Vector2 closestPoint;
            Vector2.Multiply(ref planeDirection, closestToCenterT, out closestPoint);
            Vector2.Add(ref planeOrigin, ref closestPoint, out closestPoint);
            //How close does the ray come to the circle?
            float squaredDistance = closestPoint.LengthSquared();
            if (squaredDistance > collisionMargin * collisionMargin)
            {
                //It's too far!  The ray cannot possibly hit the capsule.
                hit = default(RayHit);
                return false;
            }


            //With the squared distance, compute the distance backward along the ray from the closest point on the ray to the axis.
            float backwardsDistance = collisionMargin * (float)Math.Sqrt(1 - squaredDistance / (collisionMargin * collisionMargin));
            float tOffset = backwardsDistance / (float)Math.Sqrt(planeDirectionLengthSquared);

            hit.T = closestToCenterT - tOffset;

            //Compute the impact point on the infinite cylinder in 3d local space.
            Vector3.Multiply(ref localRay.Direction, hit.T, out hit.Location);
            Vector3.Add(ref hit.Location, ref localRay.Position, out hit.Location);

            //get growth elements
            float hitGrow; float hitSide1; float hitSide2;
            GetGrowElements(ref hit.Location, out hitGrow, out hitSide1, out hitSide2);

            //Is it intersecting the cylindrical portion of the capsule?
            if (hitGrow <= halfLength && hitGrow >= -halfLength && hit.T < maximumLength)
            {
                CreateHitNormal(hitSide1, hitSide2, out hit.Normal);
                float normalLengthSquared = hit.Normal.LengthSquared();
                if (normalLengthSquared > 1e-9f)
                    Vector3.Divide(ref hit.Normal, (float)Math.Sqrt(normalLengthSquared), out hit.Normal);
                else
                    hit.Normal = default(Vector3);
                //Pull the hit into world space.
                Quaternion.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref hit.Location, ref transform, out hit.Location);
                return true;
            }

            if (hitGrow < halfLength)
                goto lowerSphereTest;

        upperSphereTest:
            //Nope! It may be intersecting the ends of the capsule though.
            //We're above the capsule, so cast a ray against the upper sphere.
            //We don't have to worry about it hitting the bottom of the sphere since it would have hit the cylinder portion first.
            var spherePosition = new Vector3(0, halfLength, 0);
            if (Toolbox.RayCastSphere(ref localRay, ref spherePosition, collisionMargin, maximumLength, out hit))
            {
                //Pull the hit into world space.
                Quaternion.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref hit.Location, ref transform, out hit.Location);
                return true;
            }
            //No intersection! We can't be hitting the other sphere, so it's over!
            hit = default(RayHit);
            return false;

        lowerSphereTest:
            //Okay, what about the bottom sphere?
            //We're above the capsule, so cast a ray against the upper sphere.
            //We don't have to worry about it hitting the bottom of the sphere since it would have hit the cylinder portion first.
            spherePosition = new Vector3(0, -halfLength, 0);
            if (Toolbox.RayCastSphere(ref localRay, ref spherePosition, collisionMargin, maximumLength, out hit))
            {
                //Pull the hit into world space.
                Quaternion.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref hit.Location, ref transform, out hit.Location);
                return true;
            }
            //No intersection! We can't be hitting the other sphere, so it's over!
            hit = default(RayHit);
            return false;

        }

        /*
        void ChooseGrowVector()
        {
            //set up vector
            if (_Axis == Axis.X)
                posGrowVector = Toolbox.RightVector;
            else if (_Axis == Axis.Y)
                posGrowVector = Toolbox.UpVector;
            else if (_Axis == Axis.Z)
                posGrowVector = Toolbox.ForwardVector;
            else
                throw new Exception("Invalid Axis");
            //set down vector
            negGrowVector = -posGrowVector;
        }
        */

        void GetGrowElements(ref Vector3 vin, out float Grow, out float Side1, out float Side2)
        {
            if (_Axis == Axis.X)
            {
                Grow = vin.X;
                Side1 = vin.Y;
                Side2 = vin.Z;
            }
            else if (_Axis == Axis.Y)
            {
                Grow = vin.Y;
                Side1 = vin.X;
                Side2 = vin.Z;
            }
            else if (_Axis == Axis.Z)
            {
                Grow = vin.Z;
                Side1 = vin.X;
                Side2 = vin.Y;
            }
            else
                throw new Exception("Invalid Axis");
        }


        void CreateHitNormal(float Side1, float Side2, out Vector3 vout)
        {
            if (_Axis == Axis.X)
            {
                vout.X = 0;
                vout.Y = Side1;
                vout.Z = Side2;
            }
            else if (_Axis == Axis.Y)
            {
                vout.Y = 0;
                vout.X = Side1;
                vout.Z = Side2;
            }
            else if (_Axis == Axis.Z)
            {
                vout.Z = 0;
                vout.X = Side1;
                vout.Y = Side2;
            }
            else
                throw new Exception("Invalid Axis");
        }
    }
}
