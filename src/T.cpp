
#include "..\include\Utils.h"
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
		readValidLine(buffer, 200, fp);
		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
		case RB_NAME:
			//this->name = std::string() + trim(line);
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
		case RB_MESH_TRANSFORMATION: {
			/*Quaternion q;
			V3D T;
			sscanf(line, "%lf %lf %lf %lf %lf %lf %lf",
			&q[0], &q[1], &q[2], &q[3], &T[0], &T[1], &T[2]);
			meshTransformations.push_back(Transformation(q.getRotationMatrix(), T));*/
			break;
		}
		case RB_MATERIAL:			
		case RB_COLOR: 
		case RB_MASS: 
		case RB_MOI: 
		case RB_POSITION:
		case RB_ORIENTATION: 
		case RB_VELOCITY:
		case RB_ANGULAR_VELOCITY:
		case RB_FRICTION_COEFF:
		case RB_RESTITUTION_COEFF:
		case RB_END_EFFECTOR: 
		case RB_BODY_POINT_FEATURE: 
		case RB_NOT_IMPORTANT:
		case RB_CDP: 
		case RB_IS_FROZEN:
		case RB_END_RB:
		case RB_THICKNESSS:
		case RB_MESH_DESCRIPTION:
		case RB_MAPPING_INFO:
			throw("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
			break;
		default:
			throw("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
		}
	}
	throw("Incorrect articulated body input file! No /End found");
	
}

