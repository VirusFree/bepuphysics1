using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using BEPUutilities;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.Materials;
using BEPUphysics.PositionUpdating;
using System.Diagnostics;
using System.Threading;

namespace BEPUphysics.Character
{
    /// <summary>
    /// Gives a physical object FPS-like control, including stepping and jumping.
    /// This is more robust/expensive than the SimpleCharacterController.
    /// </summary>
    public class SimpleCharacterController : Updateable, IBeforeSolverUpdateable
    {
        /// <summary>
        /// Gets the physical body of the character.  Do not use this reference to modify the character's height and radius.  Instead, use the BodyRadius property and the StanceManager's StandingHeight and CrouchingHeight properties.
        /// </summary>
        public Cylinder Body { get; private set; }

        /// <summary>
        /// Gets the contact categorizer used by the character to determine how contacts affect the character's movement.
        /// </summary>
        public CharacterContactCategorizer ContactCategorizer { get; private set; }

        /// <summary>
        /// Gets the manager responsible for finding places for the character to step up and down to.
        /// </summary>
        public StepManager StepManager { get; private set; }

        /// <summary>
        /// Gets the support system which other systems use to perform local ray casts and contact queries.
        /// </summary>
        public QueryManager QueryManager { get; private set; }

        /// <summary>
        /// Gets the constraint used by the character to handle horizontal motion.  This includes acceleration due to player input and deceleration when the relative velocity
        /// between the support and the character exceeds specified maximums.
        /// </summary>
        public HorizontalMotionConstraint HorizontalMotionConstraint { get; private set; }

        /// <summary>
        /// Gets the constraint used by the character to stay glued to surfaces it stands on.
        /// </summary>
        public VerticalMotionConstraint VerticalMotionConstraint { get; private set; }

        /// <summary>
        /// Gets or sets the pair locker used by the character controller to avoid interfering with the behavior of other characters.
        /// </summary>
        private CharacterPairLocker PairLocker { get; set; }

        /// <summary>
        /// Gets or sets the down direction of the character, defining its orientation.
        /// </summary>
        public Vector3 Down
        {
            get
            {
                return Body.OrientationMatrix.Down;
            }
            set
            {
                //Update the character's orientation to something compatible with the new direction.
                Quaternion orientation;
                float lengthSquared = value.LengthSquared();
                if (lengthSquared < Toolbox.Epsilon)
                    value = Body.OrientationMatrix.Down; //Silently fail. Assuming here that a dynamic process is setting this property; don't need to make a stink about it.
                else
                    Vector3.Divide(ref value, (float)Math.Sqrt(lengthSquared), out value);
                Quaternion.GetQuaternionBetweenNormalizedVectors(ref Toolbox.DownVector, ref value, out orientation);
                Body.Orientation = orientation;
            }
        }

        float tractionForce;
        /// <summary>
        /// Gets or sets the maximum force that the character can apply while on a support which provides traction.
        /// </summary>
        public float TractionForce
        {
            get
            {
                return tractionForce;
            }
            set
            {
                tractionForce = value;
            }
        }

        public float FloatingTractionForceMultiplier = 0;

        /// <summary>
        /// Gets or sets the radius of the body cylinder.
        /// </summary>
        public float BodyRadius
        {
            get { return Body.CollisionInformation.Shape.Radius; }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Radius must be positive.");
                Body.CollisionInformation.Shape.Radius = value;
            }
        }

        /// <summary>
        /// Gets or sets the radius of the body cylinder.
        /// </summary>
        public float BodyHeight
        {
            get { return Body.CollisionInformation.Shape.Height; }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Height must be positive.");
                Body.CollisionInformation.Shape.Height = value;
            }
        }

        Vector3 viewDirection = new Vector3(0, 0, -1);
        /// <summary>
        /// Gets or sets the view direction associated with the character.
        /// Also sets the horizontal view direction internally based on the current down vector.
        /// This is used to interpret the movement directions.
        /// </summary>
        public Vector3 ViewDirection
        {
            get
            {
                return viewDirection;
            }
            set
            {
                viewDirection = value;
            }
        }


        /// <summary>
        /// Gets the support finder used by the character.
        /// The support finder analyzes the character's contacts to see if any of them provide support and/or traction.
        /// </summary>
        public SupportFinder SupportFinder { get; private set; }

        const float _JumpingDownSteppingDisableTime = 1f; //in seconds

        float _DownSteppingDisableMonitor = 0;

        /// <summary>
        /// Constructs a new character controller.
        /// </summary>
        /// <param name="position">Initial position of the character.</param>
        /// <param name="height">Height of the character body while standing.</param>
        /// <param name="radius">Radius of the character body.</param>
        /// <param name="margin">Radius of 'rounding' applied to the cylindrical body. Higher values make the cylinder's edges more rounded.
        /// The margin is contained within the cylinder's height and radius, so it must not exceed the radius or height of the cylinder.</param>
        /// <param name="mass">Mass of the character body.</param>
        /// <param name="maximumTractionSlope">Steepest slope, in radians, that the character can maintain traction on.</param>
        /// <param name="maximumSupportSlope">Steepest slope, in radians, that the character can consider a support.</param>
        /// <param name="tractionForce">Maximum force that the character can apply while on a support which provides traction.</param>
        /// <param name="maximumGlueForce">Maximum force the vertical motion constraint is allowed to apply in an attempt to keep the character on the ground.</param>
        public SimpleCharacterController(
            Vector3 position = new Vector3(),
            float height = 1.7f, float radius = 0.6f, float margin = 0.1f, float mass = 10f,
            float maximumTractionSlope = 0.8f, float maximumSupportSlope = 1.3f,
            float tractionForce = 1000,
            float maximumGlueForce = 5000
            )
        {
            if (margin > radius || margin > height)
                throw new ArgumentException("Margin must not be larger than the character's radius or height.");

            Body = new Cylinder(position, height, radius, mass);
            Body.IgnoreShapeChanges = true; //Wouldn't want inertia tensor recomputations to occur when crouching and such.
            Body.CollisionInformation.Shape.CollisionMargin = margin;
            //Making the character a continuous object prevents it from flying through walls which would be pretty jarring from a player's perspective.
            Body.PositionUpdateMode = PositionUpdateMode.Continuous;
            Body.LocalInertiaTensorInverse = new Matrix3x3();
            //TODO: In v0.16.2, compound bodies would override the material properties that get set in the CreatingPair event handler.
            //In a future version where this is changed, change this to conceptually minimally required CreatingPair.
            Body.CollisionInformation.Events.DetectingInitialCollision += RemoveFriction;
            Body.LinearDamping = 0;
            ContactCategorizer = new CharacterContactCategorizer(maximumTractionSlope, maximumSupportSlope);
            QueryManager = new QueryManager(Body, ContactCategorizer);
            SupportFinder = new SupportFinder(Body, QueryManager, ContactCategorizer);
            HorizontalMotionConstraint = new HorizontalMotionConstraint(Body, SupportFinder);
            HorizontalMotionConstraint.PositionAnchorDistanceThreshold = radius * 0.25f;
            VerticalMotionConstraint = new VerticalMotionConstraint(Body, SupportFinder, maximumGlueForce);
            StepManager = new StepManager(Body, ContactCategorizer, SupportFinder, QueryManager, HorizontalMotionConstraint);
            PairLocker = new CharacterPairLocker(Body);

            TractionForce = tractionForce;

            //Enable multithreading for the characters.  
            IsUpdatedSequentially = false;
            //Link the character body to the character controller so that it can be identified by the locker.
            //Any object which replaces this must implement the ICharacterTag for locking to work properly.
            Body.CollisionInformation.Tag = new CharacterSynchronizer(Body);
        }




        void RemoveFriction(EntityCollidable sender, BroadPhaseEntry other, NarrowPhasePair pair)
        {
            var collidablePair = pair as CollidablePairHandler;
            if (collidablePair != null)
            {
                //The default values for InteractionProperties is all zeroes- zero friction, zero bounciness.
                //That's exactly how we want the character to behave when hitting objects.
                collidablePair.UpdateMaterialProperties(new InteractionProperties());
            }
        }


        /// <summary>
        /// Cylinder shape used to compute the expanded bounding box of the character.
        /// </summary>
        void ExpandBoundingBox()
        {
            if (Body.ActivityInformation.IsActive)
            {
                //This runs after the bounding box updater is run, but before the broad phase.
                //Expanding the character's bounding box ensures that minor variations in velocity will not cause
                //any missed information.

                //TODO: seems a bit silly to do this work sequentially. Would be better if it could run in parallel in the proper location.

                var down = Down;
                var boundingBox = Body.CollisionInformation.BoundingBox;
                //Expand the bounding box up and down using the step height.
                Vector3 expansion;
                Vector3.Multiply(ref down, StepManager.MaximumStepHeight, out expansion);
                expansion.X = Math.Abs(expansion.X);
                expansion.Y = Math.Abs(expansion.Y);
                expansion.Z = Math.Abs(expansion.Z);

                //When the character climbs a step, it teleports horizontally a little to gain support. Expand the bounding box to accommodate the margin.
                //Compute the expansion caused by the extra radius along each axis.
                //There's a few ways to go about doing this.

                //The following is heavily cooked, but it is based on the angle between the vertical axis and a particular axis.
                //Given that, the amount of the radial expansion required along that axis can be computed.
                //The dot product would provide the cos(angle) between the vertical axis and a chosen axis.
                //Equivalently, it is how much expansion would be along that axis, if the vertical axis was the axis of expansion.
                //However, it's not. The dot product actually gives us the expansion along an axis perpendicular to the chosen axis, pointing away from the character's vertical axis.

                //What we need is actually given by the sin(angle), which is given by ||verticalAxis x testAxis||.
                //The sin(angle) is the projected length of the verticalAxis (not the expansion!) on the axis perpendicular to the testAxis pointing away from the character's vertical axis.
                //That projected length, however is equal to the expansion along the test axis, which is exactly what we want.
                //To show this, try setting up the triangles at the corner of a cylinder with the world axes and cylinder axes.

                //Since the test axes we're using are all standard directions ({0,0,1}, {0,1,0}, and {0,0,1}), most of the cross product logic simplifies out, and we are left with:
                var horizontalExpansionAmount = Body.CollisionInformation.Shape.CollisionMargin * 1.1f;
                Vector3 squaredDown;
                squaredDown.X = down.X * down.X;
                squaredDown.Y = down.Y * down.Y;
                squaredDown.Z = down.Z * down.Z;
                expansion.X += horizontalExpansionAmount * (float)Math.Sqrt(squaredDown.Y + squaredDown.Z);
                expansion.Y += horizontalExpansionAmount * (float)Math.Sqrt(squaredDown.X + squaredDown.Z);
                expansion.Z += horizontalExpansionAmount * (float)Math.Sqrt(squaredDown.X + squaredDown.Y);

                Vector3.Add(ref expansion, ref boundingBox.Max, out boundingBox.Max);
                Vector3.Subtract(ref boundingBox.Min, ref expansion, out boundingBox.Min);

                Body.CollisionInformation.BoundingBox = boundingBox;

            }


        }


        void IBeforeSolverUpdateable.Update(float dt)
        {
            //Someone may want to use the Body.CollisionInformation.Tag for their own purposes.
            //That could screw up the locking mechanism above and would be tricky to track down.
            //Consider using the making the custom tag implement ICharacterTag, modifying LockCharacterPairs to analyze the different Tag type, or using the Entity.Tag for the custom data instead.
            Debug.Assert(Body.CollisionInformation.Tag is ICharacterTag, "The character.Body.CollisionInformation.Tag must implement ICharacterTag to link the CharacterController and its body together for character-related locking to work in multithreaded simulations.");

            SupportData supportData;

            HorizontalMotionConstraint.UpdateMovementBasis(ref viewDirection);
            //We can't let multiple characters manage the same pairs simultaneously.  Lock it up!
            PairLocker.LockCharacterPairs();
            try
            {
                CorrectContacts();

                bool hadSupport = SupportFinder.HasSupport;

                SupportFinder.UpdateSupports(ref HorizontalMotionConstraint.movementDirection3d);
                supportData = SupportFinder.SupportData;


                //Compute the initial velocities relative to the support.
                Vector3 relativeVelocity;
                ComputeRelativeVelocity(ref supportData, out relativeVelocity);
                float verticalVelocity = Vector3.Dot(supportData.Normal, relativeVelocity);


                //Don't attempt to use an object as support if we are flying away from it (and we were never standing on it to begin with).
                if (SupportFinder.HasSupport && !hadSupport && verticalVelocity < 0)
                {
                    SupportFinder.ClearSupportData();
                    supportData = new SupportData();
                }

                //Step down
                if (_DownSteppingDisableMonitor > 0)
                    _DownSteppingDisableMonitor -= dt;
                else
                {
                    Vector3 newPosition;
                    if (StepManager.TryToStepDown(out newPosition))
                        supportData = TeleportToPosition(newPosition, dt);
                }

                //Step up
                {
                    Vector3 newPosition;
                    if (StepManager.TryToStepUp(out newPosition))
                        supportData = TeleportToPosition(newPosition, dt);
                }
            }
            finally
            {
                PairLocker.UnlockCharacterPairs();
            }

            //Tell the constraints to get ready to solve.
            HorizontalMotionConstraint.UpdateSupportData();
            VerticalMotionConstraint.UpdateSupportData();

            //Update the horizontal motion constraint's state.
            if (SupportFinder.HasSupport)
            {
                if (SupportFinder.HasTraction)
                {
                    HorizontalMotionConstraint.MovementMode = MovementMode.Traction;
                    HorizontalMotionConstraint.MaximumForce = tractionForce;
                }
                else
                {
                    HorizontalMotionConstraint.MovementMode = MovementMode.Sliding;
                    HorizontalMotionConstraint.MaximumForce = 0;
                    HorizontalMotionConstraint.TargetSpeed = 0;
                }
            }
            else
            {
                HorizontalMotionConstraint.MovementMode = MovementMode.Floating;
                HorizontalMotionConstraint.MaximumForce = tractionForce * FloatingTractionForceMultiplier;
            }

        }

        public SupportData TeleportToPosition(Vector3 newPosition, float dt)
        {

            Body.Position = newPosition;
            var orientation = Body.Orientation;
            //The re-do of contacts won't do anything unless we update the collidable's world transform.
            Body.CollisionInformation.UpdateWorldTransform(ref newPosition, ref orientation);
            //Refresh all the narrow phase collisions.
            foreach (var pair in Body.CollisionInformation.Pairs)
            {
                //Clear out the old contacts.  This prevents contacts in persistent manifolds from surviving the step
                //Such old contacts might still have old normals which blocked the character's forward motion.

                pair.ClearContacts();
                pair.UpdateCollision(dt);

            }
            //Also re-collect supports.
            //This will ensure the constraint and other velocity affectors have the most recent information available.
            SupportFinder.UpdateSupports(ref HorizontalMotionConstraint.movementDirection3d);
            return SupportFinder.SupportData;
        }

        void CorrectContacts()
        {
            //Go through the contacts associated with the character.
            //If the contact is at the bottom of the character, regardless of its normal, take a closer look.
            //If the direction from the closest point on the inner cylinder to the contact position has traction
            //and the contact's normal does not, then replace the contact normal with the offset direction.

            //This is necessary because various convex pair manifolds use persistent manifolds.
            //Contacts in these persistent manifolds can live too long for the character to behave perfectly
            //when going over (usually tiny) steps.

            Vector3 downDirection = Body.OrientationMatrix.Down;
            Vector3 position = Body.Position;
            float margin = Body.CollisionInformation.Shape.CollisionMargin;
            float minimumHeight = Body.Height * .5f - margin;
            float coreRadius = Body.Radius - margin;
            float coreRadiusSquared = coreRadius * coreRadius;
            foreach (var pair in Body.CollisionInformation.Pairs)
            {
                foreach (var contactData in pair.Contacts)
                {
                    var contact = contactData.Contact;
                    float dot;
                    //Check to see if the contact position is at the bottom of the character.
                    Vector3 offset = contact.Position - Body.Position;
                    Vector3.Dot(ref offset, ref downDirection, out dot);
                    if (dot > minimumHeight)
                    {
                        //It is a 'bottom' contact!
                        //So, compute the offset from the inner cylinder to the contact.
                        //To do this, compute the closest point on the inner cylinder.
                        //Since we know it's on the bottom, all we need is to compute the horizontal offset.
                        Vector3.Dot(ref offset, ref downDirection, out dot);
                        Vector3 horizontalOffset;
                        Vector3.Multiply(ref downDirection, dot, out horizontalOffset);
                        Vector3.Subtract(ref offset, ref horizontalOffset, out horizontalOffset);
                        float length = horizontalOffset.LengthSquared();
                        if (length > coreRadiusSquared)
                        {
                            //It's beyond the edge of the cylinder; clamp it.
                            Vector3.Multiply(ref horizontalOffset, coreRadius / (float)Math.Sqrt(length), out horizontalOffset);
                        }
                        //It's on the bottom, so add the bottom height.
                        Vector3 closestPointOnCylinder;
                        Vector3.Multiply(ref downDirection, minimumHeight, out closestPointOnCylinder);
                        Vector3.Add(ref closestPointOnCylinder, ref horizontalOffset, out closestPointOnCylinder);
                        Vector3.Add(ref closestPointOnCylinder, ref position, out closestPointOnCylinder);

                        //Compute the offset from the cylinder to the offset.
                        Vector3 offsetDirection;
                        Vector3.Subtract(ref contact.Position, ref closestPointOnCylinder, out offsetDirection);
                        length = offsetDirection.LengthSquared();
                        if (length > Toolbox.Epsilon)
                        {
                            //Normalize the offset.
                            Vector3.Divide(ref offsetDirection, (float)Math.Sqrt(length), out offsetDirection);
                        }
                        else
                            continue; //If there's no offset, it's really deep and correcting this contact might be a bad idea.

                        Vector3.Dot(ref offsetDirection, ref downDirection, out dot);
                        float dotOriginal;
                        Vector3.Dot(ref contact.Normal, ref downDirection, out dotOriginal);
                        if (dot > Math.Abs(dotOriginal)) //if the new offsetDirection normal is less steep than the original slope...
                        {
                            //Then use it!
                            Vector3.Dot(ref offsetDirection, ref contact.Normal, out dot);
                            if (dot < 0)
                            {
                                //Don't flip the normal relative to the contact normal.  That would be bad!
                                Vector3.Negate(ref offsetDirection, out offsetDirection);
                                dot = -dot;
                            }
                            //Update the contact data using the corrected information.
                            //The penetration depth is conservatively updated; it will be less than or equal to the 'true' depth in this direction.
                            contact.PenetrationDepth *= dot;
                            contact.Normal = offsetDirection;
                        }
                    }
                }
            }

        }

        void ComputeRelativeVelocity(ref SupportData supportData, out Vector3 relativeVelocity)
        {
            //Compute the relative velocity between the body and its support, if any.
            //The relative velocity will be updated as impulses are applied.
            relativeVelocity = Body.LinearVelocity;
            if (SupportFinder.HasSupport)
            {
                //Only entities have velocity.
                var entityCollidable = supportData.SupportObject as EntityCollidable;
                if (entityCollidable != null)
                {
                    //It's possible for the support's velocity to change due to another character jumping if the support is dynamic.
                    //Don't let that happen while the character is computing a relative velocity!
                    Vector3 entityVelocity;
                    bool locked = entityCollidable.Entity.IsDynamic;
                    if (locked)
                        entityCollidable.Entity.Locker.Enter();
                    try
                    {
                        entityVelocity = Toolbox.GetVelocityOfPoint(supportData.Position, entityCollidable.Entity.Position, entityCollidable.Entity.LinearVelocity, entityCollidable.Entity.AngularVelocity);
                    }
                    finally
                    {
                        if (locked)
                            entityCollidable.Entity.Locker.Exit();
                    }
                    Vector3.Subtract(ref relativeVelocity, ref entityVelocity, out relativeVelocity);
                }
            }

        }

        public void DisableDownStepping(float seconds)
        {
            _DownSteppingDisableMonitor += seconds;
        }

        /// <summary>
        /// Changes the relative velocity between the character and its support.
        /// </summary>
        /// <param name="supportData">Support data to use to jump.</param>
        /// <param name="velocityChange">Change to apply to the character and support relative velocity.</param>
        public void Jump(Vector3 Speed, float Mass)
        {
            //apply to character
            Body.LinearVelocity += Speed;
            //apply to support
            /*
            var entityCollidable = SupportFinder.SupportData.SupportObject as EntityCollidable;
            if (entityCollidable != null)
            {
                if (entityCollidable.Entity.IsDynamic)
                {
                    //Multiple characters cannot attempt to modify another entity's velocity at the same time.
                    entityCollidable.Entity.Locker.Enter();
                    try
                    {
                        entityCollidable.Entity.LinearMomentum += Speed * -Mass;
                    }
                    finally
                    {
                        entityCollidable.Entity.Locker.Exit();
                    }
                }
            }
            */
            //disable stepping for a while
            DisableDownStepping(_JumpingDownSteppingDisableTime);

            //Prevent any old contacts from hanging around and coming back with a negative depth.
            foreach (var pair in Body.CollisionInformation.Pairs)
                pair.ClearContacts();
            SupportFinder.ClearSupportData();
        }

        public override void OnAdditionToSpace(Space newSpace)
        {
            //Add any supplements to the space too.
            newSpace.Add(Body);
            newSpace.Add(HorizontalMotionConstraint);
            newSpace.Add(VerticalMotionConstraint);
            //This character controller requires the standard implementation of Space.
            newSpace.BoundingBoxUpdater.Finishing += ExpandBoundingBox;

            Body.AngularVelocity = new Vector3();
            Body.LinearVelocity = new Vector3();
        }
        public override void OnRemovalFromSpace(Space oldSpace)
        {
            //Remove any supplements from the space too.
            oldSpace.Remove(Body);
            oldSpace.Remove(HorizontalMotionConstraint);
            oldSpace.Remove(VerticalMotionConstraint);
            //This character controller requires the standard implementation of Space.
            oldSpace.BoundingBoxUpdater.Finishing -= ExpandBoundingBox;
            SupportFinder.ClearSupportData();
            Body.AngularVelocity = new Vector3();
            Body.LinearVelocity = new Vector3();
        }


    }
}

