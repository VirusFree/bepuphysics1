using BEPUphysics.Constraints.TwoEntity;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities;
using BEPUutilities;
 

namespace BEPUphysics.Constraints.SolverGroups
{
    /// <summary>
    /// Restricts three degrees of linear motion and one degree of angular motion.
    /// Acts like two hinges in immediate sequence.
    /// </summary>
    public class SphericalJoint : SolverGroup
    {
        /// <summary>
        /// Constructs a new constraint which restricts three degrees of linear freedom and one degree of twisting angular freedom between two entities.
        /// This constructs the internal constraints, but does not configure them.  Before using a constraint constructed in this manner,
        /// ensure that its active constituent constraints are properly configured.  The entire group as well as all internal constraints are initially inactive (IsActive = false).
        /// </summary>
        public SphericalJoint()
        {
            IsActive = false;
            BallSocketJoint = new BallSocketJoint();
            EllipseSwingLimit = new EllipseSwingLimit();
            Motor = new RevoluteMotor();
            Add(BallSocketJoint);
            Add(EllipseSwingLimit);
            Add(Motor);
        }


        /// <summary>
        /// Constructs a new constraint which restricts three degrees of linear freedom and one degree of twisting angular freedom between two entities.
        /// </summary>
        /// <param name="connectionA">First entity of the constraint pair.</param>
        /// <param name="connectionB">Second entity of the constraint pair.</param>
        /// <param name="anchor">Point around which both entities rotate in world space.</param>
        public SphericalJoint(Entity connectionA, Entity connectionB, Vector3 anchor, Vector3 twistAxis)
        {
            if (connectionA == null)
                connectionA = TwoEntityConstraint.WorldEntity;
            if (connectionB == null)
                connectionB = TwoEntityConstraint.WorldEntity;
            BallSocketJoint = new BallSocketJoint(connectionA, connectionB, anchor);
            EllipseSwingLimit = new EllipseSwingLimit(connectionA, connectionB, twistAxis, 0, 0);
            Motor = new RevoluteMotor(connectionA, connectionB, twistAxis);
            Motor.IsActive = false;
            Add(BallSocketJoint);
            Add(EllipseSwingLimit);
            Add(Motor);
        }

        /// <summary>
        /// Gets the ball socket joint that restricts linear degrees of freedom.
        /// </summary>
        public BallSocketJoint BallSocketJoint { get; private set; }

        /// <summary>
        /// Gets the motor of the universal joint.
        /// This constraint overlaps with the twistJoint; if the motor is activated,
        /// the twistJoint should generally be deactivated and vice versa.
        /// </summary>
        public RevoluteMotor Motor { get; private set; }

        /// <summary>
        /// Gets the angular joint which removes one twisting degree of freedom.
        /// </summary>
        public EllipseSwingLimit EllipseSwingLimit { get; private set; }
        
        /// <summary>
        /// Gets the total impulse applied by this constraint.
        /// </summary>
        public float TotalImpulse
        {
            get { return EllipseSwingLimit.TotalImpulse + BallSocketJoint.TotalImpulse.Length(); }
        }
    }
}