/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///-----includes_start-----
#include "btBulletDynamicsCommon.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
using namespace std;

vector<btVector3> first;
vector<btTransform> principal, inv;
vector<btRigidBody*> rbodies;
vector<btCompoundShape*> cbodies;

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

void bfs(int ii, int jj, int kk, char* data, vector<btVector3>&arr)
{
	queue<vector<int>> q;
	q.push({ii, jj, kk});

	while (not q.empty())
	{
		vector<int> v = q.front();
		q.pop();
		int i = v[0], j = v[1], k = v[2];

		if(i < 0 or j < 0 or k < 0 or i >= 64 or j >= 64 or k >= 64)
            continue;
        if (data[64*64*i + 64*j + k] == 0)
            continue;

        data[64*64*i + 64*j + k] = 0;
        arr.push_back(btVector3(i, j, k));

        q.push({i + 1, j, k});
        q.push({i - 1, j, k});
        q.push({i, j + 1, k});
        q.push({i, j - 1, k});
        q.push({i, j, k + 1});
        q.push({i, j, k - 1});
	}

}

void fill_pos(char* data, vector<vector<btVector3>> &pos)
{
	pos.clear();

	for (int i = 0; i < 64; i++)
		for (int j = 0; j < 64; j++)
			for (int k = 0; k < 64; k++)
				if (data[64*64*i + 64*j + k] == 1)
				{
					pos.push_back(vector<btVector3>());
					bfs(i, j, k, data, pos[pos.size()-1]);

					first.push_back(pos[pos.size()-1][0]);
					for (int i = 0; i < pos[pos.size()-1].size(); i++)
						pos[pos.size()-1][i] -= first[first.size()-1];
				}	
}


btRigidBody* add_rigid_body(btCollisionShape* col, btScalar mass, 
	                        btTransform& transform, btDiscreteDynamicsWorld* world)
{
	btVector3 localInertia(0, 0, 0);
	if (mass != 0.f)
		col->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* ms = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, ms, col, localInertia);
	cInfo.m_friction = 1.5;
	btRigidBody* body = new btRigidBody(cInfo);
	world->addRigidBody(body);

	return body;
}

int main(int argc, char** argv)
{
	///-----initialization_start-----
	btDefaultCollisionConfiguration* colCfg = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(colCfg);
	btBroadphaseInterface* brdPhse = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, brdPhse, solver, colCfg);
	dynamicsWorld->setGravity(btVector3(0, -10, 0));
	///-----initialization_end-----

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> colShapes;


	{
		btStaticPlaneShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
		colShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, 0, 0)); 
		add_rigid_body(groundShape, 0.0, groundTransform, dynamicsWorld);
	}

	vector<vector<btVector3>> pos;
	char data[64*64*64];
	read_binvox("table2.binvox", data);
	fill_pos(data, pos);
	btBoxShape* cube  = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	colShapes.push_back(cube);

	btVector3 inertia;
	principal.resize(pos.size()); inv.resize(pos.size());
	rbodies.resize(pos.size()); cbodies.resize(pos.size());

	for (int k = 0; k < pos.size(); k++)
	{
		btCompoundShape* c1 = new btCompoundShape(), *c2 = new btCompoundShape();
		btTransform transform;
		
		for (int i = 0; i < pos[k].size(); i++)
		{
			transform.setIdentity();
			transform.setOrigin(pos[k][i]);
			c1->addChildShape(transform, cube);
		}

		btScalar masses[pos[k].size()];
		for (int i = 0; i < pos[k].size(); i++) masses[i] = 1.0;
		
		c1->calculatePrincipalAxisTransform(masses, principal[k], inertia);
		inv[k] = principal[k].inverse();
		colShapes.push_back(c2);
		
		for (int i = 0; i < c1->getNumChildShapes(); i++)
			c2->addChildShape(c1->getChildTransform(i) * inv[k],
									 c1->getChildShape(i));
		delete c1;

		transform.setIdentity();
		transform.setOrigin(principal[k] * (first[k] + btVector3(0.5, 0.5, 0.5)));
		rbodies[k] = add_rigid_body(c2, 1.0, transform, dynamicsWorld);
		cbodies[k] = c2;
	}
	

	int N = dynamicsWorld->getNumCollisionObjects();

	vector<vector<btVector3>> initPos(rbodies.size());
	for (int j = 0; j < rbodies.size(); j++)
	{	
		initPos[j].resize(pos[j].size());
		for (int i = 0; i < pos[j].size(); i++)
		{
			btTransform trans;
			rbodies[j]->getMotionState()->getWorldTransform(trans);
			initPos[j][i] = (inv[j] * trans * cbodies[j]->getChildTransform(i) * principal[j]).getOrigin();			
		}
	}

	///-----stepsimulation_start-----
	for (int i = 0; ; i++)
	{
		int count = 0;

		for (int j = 0; j < N; j++)
		{
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
			if (obj->getActivationState() == ISLAND_SLEEPING)
				count++;
		}

		if (count == N)
			break;

		dynamicsWorld->stepSimulation(1.f / 50.f, 100);
	}

	btScalar score = 0, num = 0;
	for (int j = 0; j < rbodies.size(); j++)
	{	
		initPos[j].resize(pos[j].size());
		for (int i = 0; i < pos[j].size(); i++)
		{
			btTransform trans;
			rbodies[j]->getMotionState()->getWorldTransform(trans);
			btVector3 finalPos = (inv[j] * trans * cbodies[j]->getChildTransform(i) * principal[j]).getOrigin();
			score += (initPos[j][i] - finalPos).length();
			num += 1;			
		}
	}

	printf("%.2f\n", score/num);

	//remove the rigidbodies from the dynamics world and delete them
	for (int i = N - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
			delete body->getMotionState();
		dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	for (int j = 0; j < colShapes.size(); j++)
		delete colShapes[j];
	delete dynamicsWorld;
	delete solver;
	delete brdPhse;
	delete dispatcher;
	delete colCfg;
}
