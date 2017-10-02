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

	doConvexDecomposition(&gVerticesTABLE[0], 3, TABLE_NUM_VERTICES,
			gIndicesTABLE[0], 3, TABLE_NUM_TRIANGLES);

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



