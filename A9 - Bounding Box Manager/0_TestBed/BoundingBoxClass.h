/*
Programmer: Alberto Bobadilla (labigm@gmail.com)
Date: 2015/03
*/
#ifndef __BOUNDINGBOXCLASS_H_
#define __BOUNDINGBOXCLASS_H_

#include "MyEngine.h"

//System Class
class BoundingBoxClass
{
	bool m_bInitialized; //Has the elements of this object been initialized before?
	vector3 m_v3Centroid;//Center point of the model, the origin in GSpace in which the sphere is going to be located
	vector3 m_v3Min;//Collection of minimal components of the model
	vector3 m_v3Max;//Collection of maximum componets of the model
	vector3 m_v3Size;//Size of the Box
	vector3 m_v3AABBSize; //Size of the AABB
	String m_sName;//Name of the BoundingBox (to relate to the instance)
	vector3 m_v3Min2;//Collection of minimal components of the model for AABB
	vector3 m_v3Max2;//Collection of maximum componets of the model for AABB

public:
	/* Constructor */
	BoundingBoxClass(void);
	/* Copy Constructor */
	BoundingBoxClass(BoundingBoxClass const& other);
	/* Copy Assignment Operator*/
	BoundingBoxClass& operator=(BoundingBoxClass const& other);
	/* Destructor */
	~BoundingBoxClass(void);

	/* Swaps the contents of the object with another object's content */
	void Swap(BoundingBoxClass& other);

	/*
	IsInitialized
		Returns information about whether or not the object has been initialized
	*/
	bool IsInitialized(void);

	/*
	GetMinimum
		Returns the minimum vector
	*/
	vector3 GetMinimumOBB(void);

	/*
	GetMaximum
		Returns the maximum vector
	*/
	vector3 GetMaximumOBB(void);

	/*
	GetCentroid
		Returns the origin of the Cartesian Coordinate System in which the sphere is drawn in GSpace
	*/
	vector3 GetCentroid(void);

	/*
	GetName
		Returns the name of the sphere to associate it with an instance
	*/
	String GetName(void);

	/*
	GenerateOrientedBoundingBox
		Creates the Axis Aligned Bounding Box in local space of the model specified in the instance name
	*/
	void GenerateOrientedBoundingBox(String a_sInstanceName);

	/*
	GenerateAxisAlignedBoundingBox
		Creates the Oriented Bounding Box in global space
	*/
	void GenerateAxisAlignedBoundingBox(matrix4 a_m4ModeltoWorld);

	/*
	AddSphereToRenderList
		Adds the sphere to the render list specifying also if rendering the centroid is necessary
	*/
	void AddBoxToRenderList(matrix4 a_mModelToWorld, vector3 a_vColor, bool a_bRenderCentroid = false);


	vector3 aabbSize;
	vector3 aabbCenter;
	vector3 aabbColor;

private:
	/* Releases the object from memory */
	void Release(void);

	/* Initialize the object's fields */
	void Init(void);
};


#endif //__BoundingBoxClass_H__