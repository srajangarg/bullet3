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


struct SimpleBoxExample : public CommonRigidBodyBase
{
	SimpleBoxExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~SimpleBoxExample(){}
	virtual void initPhysics();
	virtual void renderScene();
	virtual void doConvexDecomposition(const btScalar* points, unsigned int stridePoints, unsigned int countPoints,
		const int* triangles, unsigned int strideTriangles, unsigned int countTriangles);
	void resetCamera()
	{
		float dist = 41;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}
};

void SimpleBoxExample::doConvexDecomposition(
	const btScalar* points, unsigned int stridePoints, unsigned int countPoints,
	const int* triangles, unsigned int strideTriangles, unsigned int countTriangles)
{
	VHACD::IVHACD* hacd = VHACD::CreateVHACD();
	VHACD::IVHACD::Parameters params = VHACD::IVHACD::Parameters();
	params.m_oclAcceleration = false;
	bool res = hacd->Compute(points, stridePoints, countPoints,
		triangles, strideTriangles, countTriangles, params);

	if (res)
	{
		// Add the resulting convex shapes to a compound shape
		btCompoundShape* compoundShape = new btCompoundShape();
		btVector3 fcentroid(0,0,0);

		unsigned int nConvexHulls = hacd->GetNConvexHulls();
		VHACD::IVHACD::ConvexHull convexHull;
		for (unsigned int ch = 0; ch < nConvexHulls; ch++)
		{
			hacd->GetConvexHull(ch, convexHull);
			unsigned int nDoubles = convexHull.m_nPoints * 3;
			std::cout<< ch << " " << convexHull.m_nPoints << std::endl;

			// Calculate centroid (center of mass)
			btVector3 centroid(0,0,0);
			for (unsigned int i = 0; i < nDoubles; i += 3)
				centroid += btVector3((btScalar)convexHull.m_points[i],
									  (btScalar)convexHull.m_points[i + 1],
						  			  (btScalar)convexHull.m_points[i + 2]);
			centroid /= (btScalar)convexHull.m_nPoints;
			if (ch == 0)
				fcentroid = centroid;

			// Create convex shape
			// Adjust points such that the centroid is at (0,0,0)
			btConvexHullShape* convexShape  = new btConvexHullShape();
			for (unsigned int i = 0; i < nDoubles; i += 3)
			{
				btVector3 point(
					(btScalar)convexHull.m_points[i],
					(btScalar)convexHull.m_points[i + 1],
					(btScalar)convexHull.m_points[i + 2]);
				point -= centroid;
				convexShape->addPoint(point, false);
			}
			convexShape->recalcLocalAabb();
			m_collisionShapes.push_back(convexShape);

			// Append to the compound shape
			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(centroid - fcentroid);
			compoundShape->addChildShape(transform, convexShape);

			// Also create a separate body for each convex shape
			// transform.setIdentity();
			// transform.setOrigin(btVector3(0,0,0));// + centroid);
			// createRigidBody(0.1f, transform, convexShape);
		}

		m_collisionShapes.push_back(compoundShape);
		btTransform tf;
		tf.setIdentity();
		tf.setOrigin(btVector3(0,10,0));
		createRigidBody(1.0 , tf, compoundShape);
	}

	hacd->Clean();
	hacd->Release();
}


const int CUBES = 154;
btVector3 pos[CUBES] = {
btVector3(0, 0, 0),
btVector3(0, 0, 1),
btVector3(0, 0, 2),
btVector3(0, 0, 3),
btVector3(0, 0, 4),
btVector3(0, 0, 5),
btVector3(0, 0, 6),
btVector3(0, 0, 7),
btVector3(0, 0, 8),
btVector3(0, 0, 9),
btVector3(1, 0, 0),
btVector3(1, 0, 1),
btVector3(1, 0, 2),
btVector3(1, 0, 3),
btVector3(1, 0, 4),
btVector3(1, 0, 5),
btVector3(1, 0, 6),
btVector3(1, 0, 7),
btVector3(1, 0, 8),
btVector3(1, 0, 9),
btVector3(2, 0, 0),
btVector3(2, 0, 1),
btVector3(2, 0, 2),
btVector3(2, 0, 3),
btVector3(2, 0, 4),
btVector3(2, 0, 5),
btVector3(2, 0, 6),
btVector3(2, 0, 7),
btVector3(2, 0, 8),
btVector3(2, 0, 9),
btVector3(3, -8, 5),
btVector3(3, -8, 6),
btVector3(3, -8, 7),
btVector3(3, -8, 8),
btVector3(3, -7, 5),
btVector3(3, -7, 6),
btVector3(3, -7, 7),
btVector3(3, -7, 8),
btVector3(3, -6, 5),
btVector3(3, -6, 6),
btVector3(3, -6, 7),
btVector3(3, -6, 8),
btVector3(3, -5, 5),
btVector3(3, -5, 6),
btVector3(3, -5, 7),
btVector3(3, -5, 8),
btVector3(3, -4, 5),
btVector3(3, -4, 6),
btVector3(3, -4, 7),
btVector3(3, -4, 8),
btVector3(3, -3, 5),
btVector3(3, -3, 6),
btVector3(3, -3, 7),
btVector3(3, -3, 8),
btVector3(3, -2, 5),
btVector3(3, -2, 6),
btVector3(3, -2, 7),
btVector3(3, -2, 8),
btVector3(3, -1, 5),
btVector3(3, -1, 6),
btVector3(3, -1, 7),
btVector3(3, -1, 8),
btVector3(3, 0, 0),
btVector3(3, 0, 1),
btVector3(3, 0, 2),
btVector3(3, 0, 3),
btVector3(3, 0, 4),
btVector3(3, 0, 5),
btVector3(3, 0, 6),
btVector3(3, 0, 7),
btVector3(3, 0, 8),
btVector3(3, 0, 9),
btVector3(4, -8, 5),
btVector3(4, -8, 6),
btVector3(4, -8, 7),
btVector3(4, -8, 8),
btVector3(4, -7, 5),
btVector3(4, -7, 6),
btVector3(4, -7, 7),
btVector3(4, -7, 8),
btVector3(4, -6, 5),
btVector3(4, -6, 6),
btVector3(4, -6, 7),
btVector3(4, -6, 8),
btVector3(4, -5, 5),
btVector3(4, -5, 6),
btVector3(4, -5, 7),
btVector3(4, -5, 8),
btVector3(4, -4, 5),
btVector3(4, -4, 6),
btVector3(4, -4, 7),
btVector3(4, -4, 8),
btVector3(4, -3, 5),
btVector3(4, -3, 6),
btVector3(4, -3, 7),
btVector3(4, -3, 8),
btVector3(4, -2, 5),
btVector3(4, -2, 6),
btVector3(4, -2, 7),
btVector3(4, -2, 8),
btVector3(4, -1, 5),
btVector3(4, -1, 6),
btVector3(4, -1, 7),
btVector3(4, -1, 8),
btVector3(4, 0, 0),
btVector3(4, 0, 1),
btVector3(4, 0, 2),
btVector3(4, 0, 3),
btVector3(4, 0, 4),
btVector3(4, 0, 5),
btVector3(4, 0, 6),
btVector3(4, 0, 7),
btVector3(4, 0, 8),
btVector3(4, 0, 9),
btVector3(5, 0, 0),
btVector3(5, 0, 1),
btVector3(5, 0, 2),
btVector3(5, 0, 3),
btVector3(5, 0, 4),
btVector3(5, 0, 5),
btVector3(5, 0, 6),
btVector3(5, 0, 7),
btVector3(5, 0, 8),
btVector3(5, 0, 9),
btVector3(6, 0, 0),
btVector3(6, 0, 1),
btVector3(6, 0, 2),
btVector3(6, 0, 3),
btVector3(6, 0, 4),
btVector3(6, 0, 5),
btVector3(6, 0, 6),
btVector3(6, 0, 7),
btVector3(6, 0, 8),
btVector3(6, 0, 9),
btVector3(7, 0, 0),
btVector3(7, 0, 1),
btVector3(7, 0, 2),
btVector3(7, 0, 3),
btVector3(7, 0, 4),
btVector3(7, 0, 5),
btVector3(7, 0, 6),
btVector3(7, 0, 7),
btVector3(7, 0, 8),
btVector3(7, 0, 9),
btVector3(8, 0, 0),
btVector3(8, 0, 1),
btVector3(8, 0, 2),
btVector3(8, 0, 3),
btVector3(8, 0, 4),
btVector3(8, 0, 5),
btVector3(8, 0, 6),
btVector3(8, 0, 7),
btVector3(8, 0, 8),
btVector3(8, 0, 9),
};


void SimpleBoxExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	// /create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0)); 
	{
		btScalar mass(0.);
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
	}

	// doConvexDecomposition(&gVerticesTABLE[0], 3, TABLE_NUM_VERTICES,
	// 		gIndicesTABLE[0], 3, TABLE_NUM_TRIANGLES);

	{
		btCompoundShape* compoundShape = new btCompoundShape();
		btBoxShape* cube  = new btBoxShape(btVector3(0.5, 0.5, 0.5));
		m_collisionShapes.push_back(cube);
		btTransform transform;
		
		// for (int i = 0; i < CUBES; i++)
		// {
		transform.setIdentity();
		transform.setOrigin(btVector3(0, 0, 0));
		compoundShape->addChildShape(transform, cube);

		transform.setIdentity();
		transform.setOrigin(btVector3(0, 1, 0));
		compoundShape->addChildShape(transform, cube);

		transform.setIdentity();
		transform.setOrigin(btVector3(0, 2, 0));
		compoundShape->addChildShape(transform, cube);

		transform.setIdentity();
		transform.setOrigin(btVector3(0, 1, 1));
		compoundShape->addChildShape(transform, cube);


		transform.setIdentity();
		transform.setOrigin(btVector3(0, 10, 0));
		btScalar masses[4]={1,1,1,1};
		btTransform principal;
		btVector3 inertia;
		compoundShape->calculatePrincipalAxisTransform(masses,principal,inertia);
		btCompoundShape* compound2 = new btCompoundShape();
		m_collisionShapes.push_back(compound2);
#if 0
		compound2->addChildShape(principal.inverse(), compoundShape);
#else	
		for (int i=0;i<compoundShape->getNumChildShapes();i++)
		{
			compound2->addChildShape(compoundShape->getChildTransform(i)*principal.inverse(),compoundShape->getChildShape(i));
		}
#endif
		delete compoundShape;
		createRigidBody(1.0 , transform, compound2);
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



