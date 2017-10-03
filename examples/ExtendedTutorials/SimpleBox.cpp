/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "SimpleBox.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "LinearMath/btIDebugDraw.h"
#include "../../Extras/VHACD/public/VHACD.h"
#include "../SoftDemo/BunnyMesh.h"
// #include "../SoftDemo/TorusMesh.h"
#include "../SoftDemo/Table.h"
#include "../SoftDemo/TableOg.h"
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;


struct SimpleBoxExample : public CommonRigidBodyBase
{
	SimpleBoxExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~SimpleBoxExample(){}
	virtual void initPhysics();
	virtual void renderScene();
	// virtual void doConvexDecomposition(const btScalar* points, unsigned int stridePoints, unsigned int countPoints,
	// 	const int* triangles, unsigned int strideTriangles, unsigned int countTriangles);
	void resetCamera()
	{
		float dist = 60;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}
};

void read_binvox(string file, char* data)
{
	ifstream infile(file.c_str());
	infile.seekg(0, ios::end);
	size_t file_size_in_byte = infile.tellg();
	infile.seekg(0, ios::beg);

	string line;
	for (int i = 0; i < 5; i++)
		getline(infile, line);
	file_size_in_byte -= infile.tellg();

	cout<<"----------"<<endl;
	cout<<file_size_in_byte<<endl;
	cout<<"----------"<<endl;
	char* buf = new char[file_size_in_byte];
	infile.read(buf, file_size_in_byte);

	int run = 0;
	for (int i = 0; i < file_size_in_byte; i += 2)
		for (int j = 0; j < (unsigned char)buf[i+1]; j++)
			data[run++] = (unsigned char)buf[i];

	char t[64 * 64];

	for (int i = 0; i < 64; i++)
	{
		for (int j = 0; j < 64; j++)
			for (int k = 0; k < 64; k++)
				t[64*j + k] = data[64*64*i + 64*j + k];

		for (int j = 0; j < 64; j++)
			for (int k = 0; k < 64; k++)
				data[64*64*i + 64*j + k] = t[64*k + j];
	}
}

void fill_pos(char* data, vector<btVector3> &pos)
{
	pos.clear();

	for (int i = 0; i < 64; i++)
		for (int j = 0; j < 64; j++)
			for (int k = 0; k < 64; k++)
				if (data[64*64*i + 64*j + k] == 1)
					pos.push_back(btVector3(i, j, k));

	btVector3 first = pos[0];
	for (int i = 0; i < pos.size(); i++)
		pos[i] -= first;
}


void SimpleBoxExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	btStaticPlaneShape*  groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, 0)); 
	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape);
	}

	vector<btVector3> pos;
	char data[64*64*64];
	read_binvox("table0.binvox", data);
	fill_pos(data, pos);

	{
		btCompoundShape* c1 = new btCompoundShape(), *c2 = new btCompoundShape();
		btBoxShape* cube  = new btBoxShape(btVector3(0.5, 0.5, 0.5));
		m_collisionShapes.push_back(cube);
		btTransform transform;
		
		for (int i = 0; i < pos.size(); i++)
		{
			transform.setIdentity();
			transform.setOrigin(pos[i]);
			c1->addChildShape(transform, cube);
		}

		btScalar masses[pos.size()];
		for (int i = 0; i < pos.size(); i++) masses[i] = 1.0;
		
		btTransform principal;
		btVector3 inertia;
		c1->calculatePrincipalAxisTransform(masses, principal, inertia);
		m_collisionShapes.push_back(c2);
		
		for (int i=0;i<c1->getNumChildShapes();i++)
			c2->addChildShape(c1->getChildTransform(i)*principal.inverse(),
									 c1->getChildShape(i));
		delete c1;

		transform.setIdentity();
		btRigidBody* bbb = createRigidBody(1.0, transform, c2);

		btVector3 aabbmin, aabbmax;
		bbb->getAabb(aabbmin, aabbmax);
		cout<<aabbmin.getX()<<" "<<aabbmin.getY()<<" "<<aabbmin.getZ()<<" "<<endl;
		cout<<aabbmax.getX()<<" "<<aabbmax.getY()<<" "<<aabbmax.getZ()<<" "<<endl;

		bbb->translate(btVector3(0, -aabbmin.getY(), 0));

		bbb->getAabb(aabbmin, aabbmax);
		cout<<aabbmin.getX()<<" "<<aabbmin.getY()<<" "<<aabbmin.getZ()<<" "<<endl;
		cout<<aabbmax.getX()<<" "<<aabbmax.getY()<<" "<<aabbmax.getZ()<<" "<<endl;
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void SimpleBoxExample::renderScene()
{
	CommonRigidBodyBase::renderScene();	
}







CommonExampleInterface*    ET_SimpleBoxCreateFunc(CommonExampleOptions& options)
{
	return new SimpleBoxExample(options.m_guiHelper);
}



