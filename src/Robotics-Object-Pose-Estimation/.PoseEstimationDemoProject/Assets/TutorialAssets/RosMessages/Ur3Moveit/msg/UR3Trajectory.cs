//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur3Moveit
{
    public class UR3Trajectory : Message
    {
        public const string RosMessageName = "ur3_moveit/UR3Trajectory";

        public Moveit.RobotTrajectory[] trajectory;

        public UR3Trajectory()
        {
            this.trajectory = new Moveit.RobotTrajectory[0];
        }

        public UR3Trajectory(Moveit.RobotTrajectory[] trajectory)
        {
            this.trajectory = trajectory;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(trajectory.Length));
            foreach(var entry in trajectory)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var trajectoryArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.trajectory= new Moveit.RobotTrajectory[trajectoryArrayLength];
            for(var i = 0; i < trajectoryArrayLength; i++)
            {
                this.trajectory[i] = new Moveit.RobotTrajectory();
                offset = this.trajectory[i].Deserialize(data, offset);
            }

            return offset;
        }

        public override string ToString()
        {
            return "UR3Trajectory: " +
            "\ntrajectory: " + System.String.Join(", ", trajectory.ToList());
        }
    }
}
