/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example flocking controller for the foot-bot.
 *
 * This controller lets a group of foot-bots flock in an hexagonal lattice towards
 * a light source placed in the arena. To flock, it exploits a generalization of the
 * well known Lennard-Jones potential. The parameters of the Lennard-Jones function
 * were chosen through a simple trial-and-error procedure on its graph.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/flocking.argos
 */

#ifndef FOOTBOT_FLOCKING_H
#define FOOTBOT_FLOCKING_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/* Vector2 definitions */
#include </home/eden/Senior_Project_Final/argos3/src/core/utility/math/vector2.h>

#include <loop_functions/flocking_loop_functions/flocking_loop_functions.h>
#include <unordered_set>
#include <cmath>
#include <queue>
#include <map>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using namespace std;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */

class CFootBotFlocking : public CCI_Controller
{

public:
   /*
    * The following variables are used as parameters for the
    * diffusion algorithm. You can set their value in the <parameters>
    * section of the XML configuration file, under the
    * <controllers><footbot_flocking_controller><parameters><diffusion>
    * section.
    */
   struct SDiffusionParams
   {
      /*
       * Maximum tolerance for the proximity reading between
       * the robot and the closest obstacle.
       * The proximity reading is 0 when nothing is detected
       * and grows exponentially to 1 when the obstacle is
       * touching the robot.
       */
      Real Delta;
      /* Angle tolerance range to go straight. */
      CRange<CRadians> GoStraightAngleRange;

      /* Constructor */
      SDiffusionParams();

      /* Parses the XML section for diffusion */
      void Init(TConfigurationNode &t_tree);
   };

   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams
   {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;
      /* Maximum tolerance for the angle between
       * the robot heading direction and
       * the closest obstacle detected. */
      CDegrees m_cAlpha;

      /* Maximum tolerance for the proximity reading between
       * the robot and the closest obstacle.
       * The proximity reading is 0 when nothing is detected
       * and grows exponentially to 1 when the obstacle is
       * touching the robot.
       */
      Real m_fDelta;
      /* Wheel speed. */
      Real m_fWheelVelocity;

      /* Angle tolerance range to go straight.
       * It is set to [-alpha,alpha]. */
      CRange<CRadians> m_cGoStraightAngleRange;
      void Init(TConfigurationNode &t_tree);
   };

   UInt32 curr_time_in_seconds;
   UInt32 last_time_in_seconds;
   std::string controllerID;
   CVector2 Target;

   /*
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><flocking>
    * section.
    */
   struct SFlockingInteractionParams
   {
      /* Target robot-robot distance in cm */
      Real TargetDistance;
      /* Gain of the Lennard-Jones potential */
      Real Gain;
      /* Exponent of the Lennard-Jones potential */
      Real Exponent;

      void Init(TConfigurationNode &t_node);
      Real GeneralizedLennardJones(Real f_distance);
   };

public:
   // Add to the public section
   std::vector<std::vector<CVector2>> m_cNodeGrid;
   std::vector<CVector2> m_cPath; // Stores the computed shortest path
   Real m_fNodeSpacing;
   CRange<Real> m_cArenaBoundsX, m_cArenaBoundsY;

   void CreateNodeGrid();
   void GreedyBestFirstSearch(const CVector2 &start, const CVector2 &goal);
   bool IsLineOfSight(const CVector2 &a, const CVector2 &b);
   bool IsObstacle(const CVector2 &node);

   // Add in the private section
   struct Node
   {
      CVector2 Position;
      Real Heuristic; // For GBFS
      bool operator<(const Node &other) const { return Heuristic > other.Heuristic; }
   };

   /* Class constructor. */
   CFootBotFlocking();
   /* Class destructor. */
   virtual ~CFootBotFlocking() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_flocking_controller> section.
    */
   virtual void Init(TConfigurationNode &t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   void SetLoopFunctions(CFlockingLoopFunctions *lf);

   int GetStatus();

   map<string, CVector2> robotPos;
   map<string, double> connectedRobots;

protected:
   /*
    * Calculates the vector to the closest light.
    */
   virtual CVector2 VectorToTarget();
   /*
    * Calculates the flocking interaction vector.
    */
   virtual CVector2 FlockingVector();

   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2 &c_heading);

private:
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator *m_pcWheels;
   /* Pointer to the foot-bot light sensor */
   // CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator *m_pcLEDs;
   /* Pointer to the omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor *m_pcCamera;

   CCI_PositioningSensor *m_compassSensor;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor *m_pcProximity;

   /* The turning parameters. */
   SWheelTurningParams m_sWheelTurningParams;
   /* The flocking interaction parameters. */
   SFlockingInteractionParams m_sFlockingParams;

   CFlockingLoopFunctions *LoopFunctions;

   /* Robot state variable */
   enum Robot_state
   {
      MOVING = 0,
      COLLIDING = 1
   } Robot_state;
};

#endif
