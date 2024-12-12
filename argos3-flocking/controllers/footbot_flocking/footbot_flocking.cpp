/* Include the controller definition */
#include "footbot_flocking.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

/************************************** */

// Method that creates a grid
void CFootBotFlocking::CreateNodeGrid()
{
   // Define the arena bounds
   m_cArenaBoundsX.Set(-15.0, 15.0); // Assuming arena size of 30x30
   m_cArenaBoundsY.Set(-15.0, 15.0);

   // Define node spacing
   m_fNodeSpacing = 1.0; // Adjust for desired resolution

   // Calculate the number of nodes along each axis
   size_t numNodesX = static_cast<size_t>((m_cArenaBoundsX.GetMax() - m_cArenaBoundsX.GetMin()) / m_fNodeSpacing) + 1;
   size_t numNodesY = static_cast<size_t>((m_cArenaBoundsY.GetMax() - m_cArenaBoundsY.GetMin()) / m_fNodeSpacing) + 1;

   // Resize the grid
   m_cNodeGrid.resize(numNodesY, std::vector<CVector2>(numNodesX));

   // Populate the grid with node positions
   for (size_t y = 0; y < numNodesY; ++y)
   {
      for (size_t x = 0; x < numNodesX; ++x)
      {
         Real nodeX = m_cArenaBoundsX.GetMin() + x * m_fNodeSpacing;
         Real nodeY = m_cArenaBoundsY.GetMin() + y * m_fNodeSpacing;
         m_cNodeGrid[y][x] = CVector2(nodeX, nodeY);
      }
   }

   LOG << "Node grid created with spacing: " << m_fNodeSpacing << std::endl;
}

void CFootBotFlocking::GreedyBestFirstSearch(const CVector2 &start, const CVector2 &goal)
{
   std::priority_queue<Node> openList;
   std::set<CVector2> visited;
   std::map<CVector2, CVector2> cameFrom;

   openList.push({start, (goal - start).Length()});

   while (!openList.empty())
   {
      Node current = openList.top();
      openList.pop();

      if (visited.count(current.Position))
         continue;
      visited.insert(current.Position);

      if ((current.Position - goal).Length() < m_fNodeSpacing)
      {
         // Goal reached, reconstruct path
         m_cPath.clear();
         for (CVector2 node = goal; node != start; node = cameFrom[node])
         {
            m_cPath.push_back(node);
         }
         m_cPath.push_back(start);
         std::reverse(m_cPath.begin(), m_cPath.end());
         return;
      }

      // Expand neighbors
      for (Real dx : std::initializer_list<Real>{-m_fNodeSpacing, 0.0, m_fNodeSpacing})
      {
         for (Real dy : std::initializer_list<Real>{-m_fNodeSpacing, 0.0, m_fNodeSpacing})
         {
            if (dx == 0.0 && dy == 0.0)
               continue;
            CVector2 neighbor = current.Position + CVector2(dx, dy);

            CVector2 neighbor = current.Position + CVector2(dx, dy);

            if (visited.count(neighbor) || !IsLineOfSight(current.Position, neighbor) || IsObstacle(neighbor))
            {
               continue;
            }

            openList.push({neighbor, (goal - neighbor).Length()});
            cameFrom[neighbor] = current.Position;
         }
      }
   }

   LOGERR << "Path not found!" << std::endl;
}

bool CFootBotFlocking::IsLineOfSight(const CVector2 &a, const CVector2 &b)
{
   // Simple line-of-sight logic assuming no obstacles block view
   // Could use Bresenham's line algorithm for grid traversal
   return true; // Placeholder for now
}

bool CFootBotFlocking::IsObstacle(const CVector2 &node)
{
   // Placeholder: Incorporate real obstacle detection using sensors
   return false;
}

/****************************************/
/****************************************/

void CFootBotFlocking::SWheelTurningParams::Init(TConfigurationNode &t_node)
{
   try
   {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);

      GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
      m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
      GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
      GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotFlocking::SFlockingInteractionParams::Init(TConfigurationNode &t_node)
{
   try
   {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotFlocking::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance)
{
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   // Real fNormDistExp = ::pow(LoopFunctions->TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

CFootBotFlocking::CFootBotFlocking() : m_pcWheels(NULL),
                                       m_pcProximity(NULL),
                                       // m_pcLight(NULL),
                                       m_pcLEDs(NULL),
                                       m_pcCamera(NULL),
                                       m_compassSensor(NULL),
                                       LoopFunctions(NULL),
                                       Robot_state(MOVING)
{
}

/****************************************/
/****************************************/

void CFootBotFlocking::Init(TConfigurationNode &t_node)
{
   LOG << "Initalization Here" << endl;
   // Call the function an startup
   CreateNodeGrid();

   CVector3 start3D = m_compassSensor->GetReading().Position; // 3D position from the compass sensor
   CVector2 start(start3D.GetX(), start3D.GetY());            // Convert to 2D

   CVector2 goal = Target; // Define goal based on experiment setup

   GreedyBestFirstSearch(start, goal);

   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the XML tag of the
    * device whose handle we want to have. For a list of allowed values, type at the
    * command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the configuration file at the
    *       <controllers><footbot_diffusion><actuators> and
    *       <controllers><footbot_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
    */
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   // m_pcLight  = GetSensor  <CCI_FootBotLightSensor                    >("footbot_light");
   m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
   m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_compassSensor = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");

   /*
    * Parse the config file
    */
   try
   {
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Flocking-related */
      m_sFlockingParams.Init(GetNode(t_node, "flocking"));
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   controllerID = GetId();
   Target = CVector2(9, 3);
   /*
    * Other init stuff
    */
   Reset();
}

void CFootBotFlocking::SetLoopFunctions(CFlockingLoopFunctions *lf)
{
   LoopFunctions = lf;
}

int CFootBotFlocking::GetStatus()
{
   return Robot_state;
}

/****************************************/
/****************************************/

void CFootBotFlocking::ControlStep()
{
   if (!m_cPath.empty())
   {
      CVector2 nextTarget = m_cPath.front();
      CVector3 currentPosition3D = m_compassSensor->GetReading().Position;
      CVector2 currentPosition(currentPosition3D.GetX(), currentPosition3D.GetY());
      CVector2 toTarget = nextTarget - currentPosition;

      if (toTarget.Length() < m_fNodeSpacing / 2)
      {
         // Target reached, remove from path
         m_cPath.erase(m_cPath.begin());
      }
      else
      {
         // Move towards target
         SetWheelSpeedsFromVector(toTarget);
      }
   }
   else
   {
      // Default behavior if path is empty
      SetWheelSpeedsFromVector(VectorToTarget() + FlockingVector());
   }
}

/****************************************/
/****************************************/

void CFootBotFlocking::Reset()
{
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);
}

/****************************************/
/****************************************/
// Method to know where and how far robot is to target
CVector2 CFootBotFlocking::VectorToTarget()
{
   CVector2 c, currPos;
   argos::CVector3 position3D = m_compassSensor->GetReading().Position;
   const argos::CQuaternion orientation = m_compassSensor->GetReading().Orientation;
   /* Convert the quaternion to Euler angles */
   argos::CRadians z_angle, y_angle, x_angle;
   orientation.ToEulerAngles(z_angle, y_angle, x_angle);

   float x = position3D.GetX();
   float y = position3D.GetY();
   currPos = CVector2(x, y);

   /* Check if the robot is near the target */
   if ((Target - currPos).Length() > 0.05)
   {
      /* Calculate heuristic distance (straight-line distance) */
      Real heuristic = (Target - currPos).Length();

      /* Check for obstacles in line of sight */
      const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
      bool obstacleDetected = false;
      for (size_t i = 0; i < tProxReads.size(); ++i)
      {
         if (tProxReads[i].Value > 0.0 && tProxReads[i].Value < 0.5)
         {
            obstacleDetected = true;
            break;
         }
      }

      /* Adjust path if obstacle is detected */
      if (obstacleDetected)
      {
         // Deviate to a point away from the obstacle
         c = CVector2(1.0, (Target - currPos).Angle() + argos::CRadians::PI_OVER_TWO);
      }
      else
      {
         // Move directly towards the target
         c = CVector2(1.0, (Target - currPos).Angle());
      }

      c.Normalize();
      c *= 0.25f * m_sWheelTurningParams.MaxSpeed;
   }
   else
   {
      c = CVector2(0, 0);
   }
   return c;
}

/****************************************/
/****************************************/

CVector2 CFootBotFlocking::FlockingVector()
{

   /* Get the camera readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &sReadings = m_pcCamera->GetReadings();

   /* Go through the camera readings to calculate the flocking interaction vector */
   if (!sReadings.BlobList.empty())
   {
      CVector2 cAccum;
      Real fLJ;
      size_t unBlobsSeen = 0;

      connectedRobots.clear();

      argos::CVector3 position3D = m_compassSensor->GetReading().Position;
      float x = position3D.GetX();
      float y = position3D.GetY();
      CVector2 pos2D = CVector2(x, y);
      float posX, posY;

      for (size_t i = 0; i < sReadings.BlobList.size(); ++i)
      {
         /*
          *  /*
          * The camera perceives the light as a yellow blob
          * The robots have their red beacon on
          * So, consider only red blobs
          * In addition: consider only the closest neighbors, to avoid
          * attraction to the farthest ones. Taking 180% of the target
          * distance is a good rule of thumb.
          */
         // if(sReadings.BlobList[i]->Color == CColor::RED &&
         //    sReadings.BlobList[i]->Distance < m_sFlockingParams.TargetDistance * 1.80f) {
         // detect neighbor robots
         // LOG  <<"ID = "<< controllerID << ", pos2D= (" << x << ", " << y<< ") "  << ", sReadings.BlobList[" << i << "]->Distance = " << sReadings.BlobList[i]->Distance << ", Angle = " << sReadings.BlobList[i]->Angle <<endl;
         // posX = x + 0.01 * sReadings.BlobList[i]->Distance * cos(sReadings.BlobList[i]->Angle.GetValue());
         // posY = y + 0.01 * sReadings.BlobList[i]->Distance * sin(sReadings.BlobList[i]->Angle.GetValue());
         // LOG <<"robot loc: ("<< posX << ", " << posY << ")" << endl;

         robotPos = LoopFunctions->GetRobotPosition();
         // Real dist=100;
         string neighborID;
         for (map<string, CVector2>::iterator it = robotPos.begin(); it != robotPos.end(); ++it)
         {
            // LOG<< "it->second="<< it->second<< ", pos2D="<< pos2D << endl;
            // LOG<<"(it->second - pos2D).Length()="<<(it->second - pos2D).Length() << ", dist="<< sReadings.BlobList[i]->Distance <<endl;
            if (abs((it->second - pos2D).Length() * 100 - sReadings.BlobList[i]->Distance) < 1.0)
            {
               // LOG<< "get the neigbor "<< it->first << endl;
               neighborID = it->first;
               connectedRobots[neighborID] = sReadings.BlobList[i]->Distance;
               break;
            }
         }
         robotPos.clear();

         // for(map<string, double>::iterator it= connectedRobots.begin(); it!= connectedRobots.end(); ++it) {
         // LOG<< "connected "<< it->first << ", dist=" << it->second << endl;
         // }

         if (sReadings.BlobList[i]->Color == CColor::RED &&
             sReadings.BlobList[i]->Distance < LoopFunctions->GetTargetDistance() * 1.80f)
         {

            /*
             * Take the blob distance and angle
             * With the distance, calculate the Lennard-Jones interaction force
             * Form a 2D vector with the interaction force and the angle
             * Sum such vector to the accumulator
             */
            /* Calculate LJ */
            fLJ = m_sFlockingParams.GeneralizedLennardJones(sReadings.BlobList[i]->Distance);
            /* Sum to accumulator */
            cAccum += CVector2(fLJ, sReadings.BlobList[i]->Angle);
            /* Increment the blobs seen counter */
            ++unBlobsSeen;
         }
      }
      if (unBlobsSeen > 0)
      {
         /* Divide the accumulator by the number of blobs seen */
         cAccum /= unBlobsSeen;
         /* Clamp the length of the vector to the max speed */
         if (cAccum.Length() > m_sWheelTurningParams.MaxSpeed)
         {
            cAccum.Normalize();
            cAccum *= m_sWheelTurningParams.MaxSpeed;
         }
         return cAccum;
      }
      else
         return CVector2();
   }
   else
   { // blob sensor empty
      return CVector2();
   }
}

/****************************************/
/****************************************/

void CFootBotFlocking::SetWheelSpeedsFromVector(const CVector2 &c_heading)
{
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN)
   {
      if (Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN)
   {
      if (Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if (Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN)
   {
      if (Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if (Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch (m_sWheelTurningParams.TurningMechanism)
   {
   case SWheelTurningParams::NO_TURN:
   {
      /* Just go straight */
      fSpeed1 = fBaseAngularWheelSpeed;
      fSpeed2 = fBaseAngularWheelSpeed;
      break;
   }
   case SWheelTurningParams::SOFT_TURN:
   {
      /* Both wheels go straight, but one is faster than the other */
      Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
      fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
      fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
      break;
   }
   case SWheelTurningParams::HARD_TURN:
   {
      /* Opposite wheel speeds */
      fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
      fSpeed2 = m_sWheelTurningParams.MaxSpeed;
      break;
   }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if (cHeadingAngle > CRadians::ZERO)
   {
      /* Turn Left */
      fLeftWheelSpeed = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else
   {
      /* Turn Right */
      fLeftWheelSpeed = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotFlocking, "footbot_flocking_controller")
