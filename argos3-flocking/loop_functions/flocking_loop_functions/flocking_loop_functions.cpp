#include "flocking_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_flocking/footbot_flocking.h>

/****************************************/
/****************************************/

CFlockingLoopFunctions::CFlockingLoopFunctions() :
   m_cFlockingArenaSideX(-0.9f, 1.2f),
   m_cFlockingArenaSideY(-1.7f, 1.7f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   DrawIDs(1)
   //m_unCollectedFood(0),
   //m_nEnergy(0),
   //m_unEnergyPerFoodItem(1),
   //m_unEnergyPerWalkingRobot(1) 
   {
}

/****************************************/
/****************************************/

void CFlockingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tFlocking = GetNode(t_node, "flocking");
     // /* Get a pointer to the floor entity */
     // m_pcFloor = &GetSpace().GetFloorEntity();
     // /* Get the number of food items we want to be scattered from XML */
     // UInt32 unFoodItems;
      GetNodeAttribute(tFlocking, "target_distance", target_distance);
     
     // /* Get the number of food items we want to be scattered from XML */
     // GetNodeAttribute(tFlocking, "radius", m_fFoodSquareRadius);
     // m_fFoodSquareRadius *= m_fFoodSquareRadius;
     // /* Create a new RNG */
     // m_pcRNG = CRandom::CreateRNG("argos");
     // /* Distribute uniformly the items in the environment */
     // for(UInt32 i = 0; i < unFoodItems; ++i) {
      //   m_cFoodPos.push_back(
     //       CVector2(m_pcRNG->Uniform(m_cFlockingArenaSideX),
     //                m_pcRNG->Uniform(m_cFlockingArenaSideY)));
     // }
     
      /* Get the output file name from XML */
     // GetNodeAttribute(tFlocking, "output", m_strOutput);
     // /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
     // /* Get energy gain per item collected */
     // GetNodeAttribute(tFlocking, "energy_per_item", m_unEnergyPerFoodItem);
     // /* Get energy loss per walking robot */
     // GetNodeAttribute(tFlocking, "energy_per_walking_robot", m_unEnergyPerWalkingRobot);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
   
   argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	   argos::CSpace::TMapPerType::iterator it;
    
    argos::LOG<<"Number of robots="<<footbots.size()<< ", DrawIDs="<< DrawIDs<<endl;
	   for(it = footbots.begin(); it != footbots.end(); it++) {
   	   	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		      //BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		      //CFootBotFlocking& c2 = dynamic_cast<CFootBotFlocking&>(c);
		      CFootBotFlocking& c2 = dynamic_cast<CFootBotFlocking&>(footBot.GetControllableEntity().GetController());
		      
        c2.SetLoopFunctions(this);
	    }   
	    
	    last_time_in_seconds = 0;
}

/****************************************/
/****************************************/

void CFlockingLoopFunctions::Reset() {
   /* Zero the counters */
   //m_unCollectedFood = 0;
   //m_nEnergy = 0;
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   /* Distribute uniformly the items in the environment */
   //for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
   //   m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cFlockingArenaSideX),
   //                     m_pcRNG->Uniform(m_cFlockingArenaSideY));
  // }
}

/****************************************/
/****************************************/

void CFlockingLoopFunctions::Destroy() {
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CFlockingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(c_position_on_plane.GetX() < -1.0f) {
      return CColor::GRAY50;
   }
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}


Real CFlockingLoopFunctions::getSimTimeInSeconds() {
	int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick(); //qilu 02/06/2021
	float sim_time = GetSpace().GetSimulationClock();
	return sim_time/ticks_per_second;
}


map<string, CVector2> CFlockingLoopFunctions::GetRobotPosition()
{
	return robotPos;
	}


void CFlockingLoopFunctions::CheckConnectivity()
{
	
	}
	
	
/****************************************/
/****************************************/
void CFlockingLoopFunctions::IncreaseTargetDistance()
{
	target_distance++;
	}

UInt32 CFlockingLoopFunctions::GetTargetDistance()
{
	return target_distance;
	}

/****************************************/
/****************************************/

void CFlockingLoopFunctions::PreStep() {
   /* Logic to pick and drop food items */
   /*
    * If a robot is in the nest, drop the food item
    * If a robot is on a food item, pick it
    * Each robot can carry only one food item per time
    */
   //UInt32 unWalkingFBs = 0;
   //UInt32 unRestingFBs = 0;
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   robotPos.clear();
   bool collide_flag = 0;
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
	
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotFlocking& cController = dynamic_cast<CFootBotFlocking&>(cFootBot.GetControllableEntity().GetController());
     
      /* Get the position of the foot-bot on the ground as a CVector2 */
      if(cController.GetStatus() == 1){
		  collide_flag = 1;
		  }
      
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      robotPos[cController.GetId()] = cPos;  
      
      //LOG << "ID="<< cController.GetId() << ", pos = " << cPos << endl;
     connectedList.clear();
     for(map<string, double>::iterator itr= cController.connectedRobots.begin(); itr!= cController.connectedRobots.end(); ++itr) {
	   //LOG<< "connected "<< it->first << endl;
	   connectedList[cController.GetId()].push_back(itr->first);
	   }  
   }
   
   LOG<< "display connected robots ..." << endl;
   for(map<string, vector<string> >::iterator itr= connectedList.begin(); itr!= connectedList.end(); ++itr){
	   LOG<< itr->first << " connects to " <<endl;
	   for(size_t i = 0;  i < itr->second.size(); i++){
		   LOG << itr->second[i]<< endl;
		   }
	   }
   
   /* Get handle to the first foot-bot entity and controller */
   CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(m_cFootbots.begin()->second);
   CFootBotFlocking& cController = dynamic_cast<CFootBotFlocking&>(cFootBot.GetControllableEntity().GetController());
   
   set<string> visited;
   string rid;
   vector<string> IDStack;
   visited.insert(cController.GetId());
   
   for(map<string, double>::iterator itr= cController.connectedRobots.begin(); itr!= cController.connectedRobots.end(); ++itr) {
	  auto tt = visited.find(itr->first);
	  if (tt == visited.end()) {
		   // insert to the stack
		   IDStack.push_back(itr->first);
		  }
	 }   
   //LOG<< "selected ID="<< cController.GetId() << ", connected "<< it->first << endl;
   while(!IDStack.empty()){
		rid = IDStack.back();
		IDStack.pop_back();
		visited.insert(rid);
		for(size_t i =0; i < connectedList[rid].size(); i++){
			auto tt = visited.find(connectedList[rid][i]);
			if(tt == visited.end()) {
				// insert to the stack
				IDStack.push_back(connectedList[rid][i]);
			}
		} 
   }  
   for (set<string>::iterator itr = visited.begin(); itr != visited.end(); ++itr) {
        std::cout <<"visited id " << *itr << " "<<endl;
    }
      
   //increase the distance to neighbors
   curr_time_in_seconds = getSimTimeInSeconds();
      if(collide_flag == 1 && curr_time_in_seconds - last_time_in_seconds >= 2 && curr_time_in_seconds > 0){
      //LOG<<"current time = "<< curr_time_in_seconds<< endl;
      if(target_distance < 200){
			IncreaseTargetDistance();
      }
      //LOG<<"TargetDistance = "<< target_distance<<endl;
      last_time_in_seconds = curr_time_in_seconds;
      }

    // check the connectivity of the swarm
    CheckConnectivity();
	
  // /* Output stuff to file */
   m_cOutput << GetSpace().GetSimulationClock() << "\t"
    //         << unWalkingFBs << "\t"
    //         << unRestingFBs << "\t"
    //         << m_unCollectedFood << "\t"
    //         << m_nEnergy 
    << std::endl;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CFlockingLoopFunctions, "flocking_loop_functions")
