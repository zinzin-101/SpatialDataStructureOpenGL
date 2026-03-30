#pragma once
const float CUBE_VERTICES[8 * 6] = {
	// positions          // normals
	-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
	 0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
	 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
	-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

	-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
	 0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
	 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
	-0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
};

const unsigned int CUBE_INDICES[6 * 6] = {
	// Right
	1, 2, 6,
	6, 5, 1,
	// Left
	0, 4, 7,
	7, 3, 0,
	// Top
	4, 5, 6,
	6, 7, 4,
	// Bottom
	0, 3, 2,
	2, 1, 0,
	// Back
	0, 1, 5,
	5, 4, 0,
	// Front
	3, 7, 6,
	6, 2, 3
};

const float PLANE_VERTICES[4 * 6] = {
	// positions			// normals
	 0.5f, 0.0f,  0.5f,		0.0f, 1.0f, 0.0f,
	 0.5f, 0.0f, -0.5f,		0.0f, 1.0f, 0.0f, 
	-0.5f, 0.0f, -0.5f,		0.0f, 1.0f, 0.0f, 
	-0.5f, 0.0f,  0.5f,		0.0f, 1.0f, 0.0f
};

const unsigned int PLANE_INDICES[6] = {
	0, 3, 1, 
	1, 3, 2  
};

