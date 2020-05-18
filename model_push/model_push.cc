/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{

class ModelPush : public ModelPlugin
{
int i = 0;
int estado;
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        model = _parent;

        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
    }

    void state(int estado){

	switch (estado)
	{
	case 1:
		model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.8));
		model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
		break;
	case 2:
		model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
		model->SetLinearVel(ignition::math::Vector3d(0, 0.6, 0));
		break;	
	case 3:
		model->SetAngularVel(ignition::math::Vector3d(0, 0, -0.45));
		model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
		break;	
	case 4:
		model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
		model->SetLinearVel(ignition::math::Vector3d(0.6, 0, 0));
		break;
	default: 
		model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
		model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
		break;
	}

	

    }

	



    // Called by the world update start event
    void OnUpdate()
    {
        // Apply a small linear velocity to the model.
	i++;
	if ((i>0)&(i<=7000)){
        	estado = 1;
	}
	else if ((i>7000)&(i<=15000)){
        	estado = 2;
	}
	else if ((i>15000)&(i<=22000)){
        	estado = 3;
	}
	else if ((i>22000)&(i<30000)){
        	estado = 4;
	}
	else estado = 0;
	state(estado);

	 printf("At: %d\n", i);

		

        

	
        ignition::math::Pose3d pose = model->WorldPose();

       

        /*math::Pose pose = model->GetWorldPose();

        printf("At: %f %f %f\n", pose.pos.x, pose.pos.y, pose.pos.z);*/
    }

private:
    physics::ModelPtr model; // Pointer to the model
    event::ConnectionPtr updateConnection; // Pointer to the update event connection
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
