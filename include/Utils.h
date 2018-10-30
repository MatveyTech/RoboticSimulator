#pragma once

#include <fstream>
#include <string>

std::string readValidLine(std::ifstream& stream); 

typedef struct key_word {
	char keyWord[50];
	int retVal;
}KeyWord;

/**
This method returns a pointer to the first non-white space character location in the provided buffer
*/
inline char* lTrim(char* buffer) {
	while (*buffer == ' ' || *buffer == '\t' || *buffer == '\n' || *buffer == '\r')
		buffer++;
	return buffer;
}

inline char* rTrim(char* buffer) {
	int index = (int)strlen(buffer) - 1;
	while (index >= 0) {
		if (buffer[index] == ' ' || buffer[index] == '\t' || buffer[index] == '\n' || buffer[index] == '\r') {
			buffer[index] = '\0';
			index--;
		}
		else
			break;
	}
	return buffer;
}

inline char* trim(char* buffer) {
	return rTrim(lTrim(buffer));
}

inline bool isWhiteSpace(char ch) {
	return (ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r' || ch == '\0');
}

/**
	This method reads a line from a file. It does not return empty lines or ones that start with a pound key - those are assumed to be comments.
	This method returns true if a line is read, false otherwise (for instance the end of file is met).
**/
inline bool readValidLine(char* line, int nChars, FILE* fp){
	line[0] = '\0';
	while (!feof(fp)){
		fgets(line, nChars, fp);
		if ((int)strlen(line)>=nChars)
			throw("The input file contains a line that is longer than the buffer - not allowed");
		char* tmp = trim(line);
		if (tmp[0]!='#' && tmp[0]!='\0')
			return true;
	}

	return false;
}
// given a list of keywords that map strings to integer values denoting keyword types, this method will determine the type of command that is passed in
int getLineType(char* &line, KeyWord* keywords, int nKeywords) {
	for (int i = 0; i < nKeywords; i++) {
		if (strncmp(line, keywords[i].keyWord, strlen(keywords[i].keyWord)) == 0 && isWhiteSpace(line[strlen(keywords[i].keyWord)])) {
			line += strlen(keywords[i].keyWord);
			return keywords[i].retVal;
		}
	}
	return -1;
}

enum RB_KEYWORDS {
	RB_NOT_IMPORTANT = -1,
	RB_RB,
	RB_END_RB,
	RB_NAME,
	RB_MASS,
	RB_MOI,
	RB_POSITION,
	RB_ORIENTATION,
	RB_VELOCITY,
	RB_ANGULAR_VELOCITY,
	RB_FRICTION_COEFF,
	RB_RESTITUTION_COEFF,
	RB_IS_FROZEN,
	RB_CDP,
	RB_MESH_NAME,
	RB_MATERIAL,
	RB_MATERIAL_DEFINITION,
	RB_COLOR,
	RB_CHILD,
	RB_PARENT,
	RB_PPOS,
	RB_CPOS,
	RB_HINGE_JOINT,
	RB_UNIVERSAL_JOINT,
	RB_BALL_AND_SOCKET_JOINT,
	RB_WELDED_JOINT,
	RB_JOINT_LIMITS,
	RB_JOINT_ROT_AXES,
	RB_JOINT_CONTROL_MODE,
	RB_JOINT_END,
	RB_END_EFFECTOR,
	RB_BODY_POINT_FEATURE,
	RB_THICKNESSS,
	RB_MOTOR_ID,
	RB_FLIPMOTORAXISDIR,
	RB_MESH_TRANSFORMATION,
	RB_MAPPING_INFO,
	RB_MESH_DESCRIPTION,
	RB_DEFAULT_ANGLE,
	RB_COMPLIANT_JOINT,
	RB_COMPLIANT_JOINT_STIFFNESS
};

KeyWord RBkeywords[] = {
	{ "RigidBody", RB_RB },
	{ "/End", RB_END_RB },
	{ "name", RB_NAME },
	{ "mass", RB_MASS },
	{ "moi", RB_MOI },
	{ "position", RB_POSITION },
	{ "orientation", RB_ORIENTATION },
	{ "velocity", RB_VELOCITY },
	{ "angularVelocity", RB_ANGULAR_VELOCITY },
	{ "frictionCoefficient", RB_FRICTION_COEFF },
	{ "restitutionCoefficient", RB_RESTITUTION_COEFF },
	{ "frozen", RB_IS_FROZEN },
	{ "CDP", RB_CDP },
	{ "mesh", RB_MESH_NAME },
	{ "material", RB_MATERIAL },
	{ "defineMaterial", RB_MATERIAL_DEFINITION },
	{ "colour", RB_COLOR },
	{ "child", RB_CHILD },
	{ "parent", RB_PARENT },
	{ "jointPPos", RB_PPOS },
	{ "jointCPos", RB_CPOS },
	{ "hingeJoint", RB_HINGE_JOINT },
	{ "universalJoint", RB_UNIVERSAL_JOINT },
	{ "ballAndSocketJoint", RB_BALL_AND_SOCKET_JOINT },
	{ "weldedJoint", RB_WELDED_JOINT },
	{ "jointLimits", RB_JOINT_LIMITS },
	{ "jointAxes", RB_JOINT_ROT_AXES },
	{ "controlMode", RB_JOINT_CONTROL_MODE },
	{ "/Joint", RB_JOINT_END },
	{ "endEffector", RB_END_EFFECTOR },
	{ "bodyPointFeature", RB_BODY_POINT_FEATURE },
	{ "thickness", RB_THICKNESSS },
	{ "motorID", RB_MOTOR_ID },
	{ "flipMotorAxis", RB_FLIPMOTORAXISDIR },
	{ "meshTransformation", RB_MESH_TRANSFORMATION },
	{ "mappingInfo", RB_MAPPING_INFO },
	{ "meshDescription", RB_MESH_DESCRIPTION },
	{ "defaultAngle", RB_DEFAULT_ANGLE },
	{ "compliantJoint",RB_COMPLIANT_JOINT },
	{ "stiffness", RB_COMPLIANT_JOINT_STIFFNESS }
};

// given a list of keywords that map strings to integer values denoting keyword types, this method will determine the string corresponding to the token passed in
char* getKeyword(int lineType, KeyWord* keywords, int nKeywords) {
	for (int i = 0; i<nKeywords; i++) {
		if (lineType == keywords[i].retVal)
			return keywords[i].keyWord;
	}
	return NULL;
}

// determine the type of a line that was used in the input file for a rigid body
int getRBLineType(char* &buffer) {
	return getLineType(buffer, RBkeywords, sizeof(RBkeywords) / sizeof(RBkeywords[0]));
}

// returns the string associated with the given token
char* getRBString(int token) {
	return getKeyword(token, RBkeywords, sizeof(RBkeywords) / sizeof(RBkeywords[0]));
}
