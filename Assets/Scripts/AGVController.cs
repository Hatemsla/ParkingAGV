using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS};

    public class AGVController : MonoBehaviour
    {
        public ArticulationBody moveWheel1;
        public ArticulationBody moveWheel2;
        public ArticulationBody moveWheel3;
        public ArticulationBody moveWheel4;
        public ArticulationBody rotationWheel1;
        public ArticulationBody rotationWheel2;
        public ArticulationBody rotationWheel3;
        public ArticulationBody rotationWheel4;
        public ControlMode mode = ControlMode.ROS;

        public float maxLinearSpeed = 2; //  m/s
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.033f; //meters
        public float trackWidth = 0.288f; // meters Distance between tyres
        public float forceLimit = 10;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        private void Start()
        {
            SetParameters(moveWheel1);
            SetParameters(moveWheel2);
            SetParameters(moveWheel3);
            SetParameters(moveWheel4);
            SetParameters(rotationWheel1);
            SetParameters(rotationWheel2);
            SetParameters(rotationWheel3);
            SetParameters(rotationWheel4);
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
        }

        private void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
        }

        private void FixedUpdate()
        {
            if (mode == ControlMode.Keyboard)
            {
                KeyBoardUpdate();
            }
            else if (mode == ControlMode.ROS)
            {
                ROSUpdate();
            }     
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }
        
        private void KeyBoardUpdate()
        {
            float moveDirection = Input.GetAxis("Vertical");
            float inputSpeed;
            float inputRotationSpeed;
            if (moveDirection > 0)
            {
                inputSpeed = maxLinearSpeed;
            }
            else if (moveDirection < 0)
            {
                inputSpeed = maxLinearSpeed * -1;
            }
            else
            {
                inputSpeed = 0;
            }

            RobotMoveInput(inputSpeed);

            float turnDirction = Input.GetAxis("Horizontal");
            if (turnDirction > 0)
            {
                inputRotationSpeed = maxRotationalSpeed;
            }
            else if (turnDirction < 0)
            {
                inputRotationSpeed = maxRotationalSpeed * -1;
            }
            else
            {
                inputRotationSpeed = 0;
            }
            RobotRotationInput(inputRotationSpeed);
        }

        private void RobotMoveInput(float speed)
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            
            float wheel1Move = (speed / wheelRadius);
            float wheel2Move = wheel1Move;
            float wheel3Move = wheel1Move;
            float wheel4Move = wheel1Move;
            float wheelSpeedDiff = ((speed * trackWidth) / wheelRadius);
            
            if (speed != 0)
            {
                wheel1Move *= Mathf.Rad2Deg;
                wheel2Move *= Mathf.Rad2Deg;
                wheel3Move *= Mathf.Rad2Deg;
                wheel4Move *= Mathf.Rad2Deg;
            }
            else
            {
                wheel1Move *= 0;
                wheel2Move *= 0;
                wheel3Move *= 0;
                wheel4Move *= 0;
            }
            
            SetSpeed(moveWheel1, wheel1Move);
            SetSpeed(moveWheel2, wheel2Move);
            SetSpeed(moveWheel3, wheel3Move);
            SetSpeed(moveWheel4, wheel4Move);
        }


        private void ROSUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            // RobotRotationInput(rosLinear, -rosAngular);
        }

        private void RobotRotationInput(float rotSpeed) // m/s and rad/s
        {
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }
            float wheel1Rotation = (rotSpeed / wheelRadius);
            float wheel2Rotation = wheel1Rotation;
            float wheel3Rotation = wheel1Rotation;
            float wheel4Rotation = wheel1Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wheel3Rotation = (wheel3Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wheel4Rotation = (wheel4Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            }
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
                wheel3Rotation *= Mathf.Rad2Deg;
                wheel4Rotation *= Mathf.Rad2Deg;
            }
            SetSpeed(rotationWheel1, wheel1Rotation);
            SetSpeed(rotationWheel2, wheel2Rotation);
            SetSpeed(rotationWheel3, wheel3Rotation);
            SetSpeed(rotationWheel4, wheel4Rotation);
        }
    }
}