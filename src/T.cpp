
#include "..\include\T.h"



T::T(void){

}

void T::loadFromFile(FILE* fp) {
	if (fp == NULL)
		throw("Invalid file pointer.");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];

	//this is where it happens.
	while (!feof(fp)) {
		//get a line from the file...
		/*readValidLine(buffer, 200, fp);
		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
		case RB_NAME:
			this->name = std::string() + trim(line);
			break;
		case RB_MESH_NAME: {
			char tmpStr[200];
			sscanf(line, "%s", tmpStr);
			std::string str(tmpStr);
			//if (str != "None") {
			//tmpMesh = GLContentManager::getGLMesh(tmpStr);
			//tmpMesh->computeNormals();
			//tmpMesh->computeTangents();
			//meshes.push_back(tmpMesh);
			//}		
		}
						   break;
		case RB_MATERIAL:
			if (meshes.size() > 0) {
				char tmpStr[200];
				sscanf(line, "%s", tmpStr);
				GLShaderMaterial* shaderMaterial = GLContentManager::getShaderMaterial(tmpStr);
				if (shaderMaterial == NULL)
					Logger::consolePrint("RigidBody::loadFromFile: specified material not found...\n");
				else
					meshes[meshes.size() - 1]->setMaterial(*shaderMaterial);
			}
			else
				Logger::consolePrint("RigidBody::loadFromFile: Warning, specified material does not apply to any mesh\n");
			break;
		case RB_COLOR: {
			double r, g, b, a;
			if (sscanf(line, "%lf %lf %lf %lf", &r, &g, &b, &a) != 4)
				throwError("Incorrect rigid body input file - colour parameter expects 4 arguments (colour %s)\n", line);
			if (meshes.size() > 0)
				meshes[meshes.size() - 1]->getMaterial().setColor(r, g, b, a);
		}
					   break;
		case RB_MASS: {
			double t = 1;
			if (sscanf(line, "%lf", &t) != 1)
				Logger::consolePrint("Incorrect rigid body input file - a mass needs to be specified if the 'mass' keyword is used.");
			this->rbProperties.mass = t;
		}
					  break;
		case RB_MOI: {
			double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;
			sscanf(line, "%lf %lf %lf %lf %lf %lf", &t1, &t2, &t3, &t4, &t5, &t6);
			if (t1 <= 0 || t2 <= 0 || t3 <= 0)
				Logger::consolePrint("Incorrect values for the principal moments of inertia.");
			this->rbProperties.setMOI(t1, t2, t3, t4, t5, t6);
		}
					 break;
		case RB_POSITION:
			if (sscanf(line, "%lf %lf %lf", &state.position[0], &state.position[1], &state.position[2]) != 3)
				throwError("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates position of a rigid body\n", line);
			break;
		case RB_ORIENTATION: {
			double t = 0, t1 = 0, t2 = 0, t3 = 0;
			if (sscanf(line, "%lf %lf %lf %lf", &t, &t1, &t2, &t3) != 4)
				throwError("Incorrect rigid body input file - 4 arguments are required to specify the world coordinates orientation of a rigid body\n", line);
			state.orientation.setRotationFrom(t, V3D(t1, t2, t3).toUnit());
		}
							 break;
		case RB_VELOCITY:
			if (sscanf(line, "%lf %lf %lf", &state.velocity[0], &state.velocity[1], &state.velocity[2]) != 3)
				throwError("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates velocity of a rigid body\n", line);
			break;
		case RB_ANGULAR_VELOCITY:
			if (sscanf(line, "%lf %lf %lf", &state.angularVelocity[0], &state.angularVelocity[1], &state.angularVelocity[2]) != 3)
				throwError("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates angular velocity of a rigid body\n", line);
			break;
		case RB_FRICTION_COEFF:
			if (sscanf(line, "%lf", &rbProperties.frictionCoeff) != 1)
				throwError("Incorrect rigid body input file - Expecting a value for the friction coefficient");
			if (rbProperties.frictionCoeff<0)
				throwError("Incorrect rigid body input file - Friction coefficient should be >= 0");
			break;
		case RB_RESTITUTION_COEFF:
			if (sscanf(line, "%lf", &rbProperties.restitutionCoeff) != 1)
				throwError("Incorrect rigid body input file - Expecting a value for the restitution coefficient");
			if (rbProperties.restitutionCoeff<0 || rbProperties.restitutionCoeff>1)
				throwError("Incorrect rigid body input file - restitution coefficient should be between 0 and 1");
			break;
		case RB_END_EFFECTOR: {
			RBEndEffector tmpEE(P3D(), 0.01);
			sscanf(line, "%lf %lf %lf %lf %d %lf %lf %lf %d", &tmpEE.coords[0], &tmpEE.coords[1], &tmpEE.coords[2], &tmpEE.featureSize, &tmpEE.eeType, &tmpEE.localCoordsWheelAxis[0], &tmpEE.localCoordsWheelAxis[1], &tmpEE.localCoordsWheelAxis[2], &tmpEE.meshIndex);
			rbProperties.endEffectorPoints.push_back(tmpEE);
		}
							  break;
		case RB_BODY_POINT_FEATURE: {
			P3D p;
			double featureSize = 0.02;
			sscanf(line, "%lf %lf %lf %lf", &p[0], &p[1], &p[2], &featureSize);
			//					if (sscanf(line, "%lf %lf %lf", &p[0], &p[1], &p[2], &featureSize) != 3)
			//						throwError("Incorrect rigid body input file - 3 arguments are required to specify a body point feature\n", line);
			rbProperties.bodyPointFeatures.push_back(RBFeaturePoint(p, featureSize));
		}
									break;
		case RB_NOT_IMPORTANT:
			if (strlen(line) != 0 && line[0] != '#')
				Logger::consolePrint("Ignoring input line: \'%s\'\n", line);
			break;
		case RB_CDP: {
			CollisionDetectionPrimitive* newCDP = CollisionDetectionPrimitive::getCDPFromDefinition(lTrim(line));
			if (newCDP == NULL)
				Logger::consolePrint("Could not load CDP from definition string: %s\n", line);
			else
				cdps.push_back(newCDP);
		}
					 break;
		case RB_IS_FROZEN:
			this->rbProperties.isFrozen = true;
			break;
		case RB_END_RB:
			if (meshDescriptions.size() != meshes.size())
				meshDescriptions.resize(meshes.size());
			return;//and... done
			break;
		case RB_THICKNESSS:
			if (sscanf(line, "%lf", &rbProperties.thickness) != 1)
				throwError("Incorrect rigid body input file - Expecting a value for the thickness coefficient");
			break;
		case RB_MESH_DESCRIPTION: {
			char tmpStr[200];
			sscanf(line, "%s", tmpStr);
			std::string str(tmpStr);
			meshDescriptions.push_back(str);
		}
								  break;
		case RB_MESH_TRANSFORMATION: {
			Quaternion q;
			V3D T;
			sscanf(line, "%lf %lf %lf %lf %lf %lf %lf",
				&q[0], &q[1], &q[2], &q[3], &T[0], &T[1], &T[2]);
			meshTransformations.push_back(Transformation(q.getRotationMatrix(), T));
			break;
		}
		case RB_MAPPING_INFO:
			sscanf(line, "%d %d", &mappingInfo.index1, &mappingInfo.index2);
			break;
		default:
			throwError("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
		}*/
	}
	throw("Incorrect articulated body input file! No /End found");
	
}

