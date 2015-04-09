#include "BoundingBoxClass.h"
//  BoundingBoxClass
void BoundingBoxClass::Init(void)
{
	m_bInitialized = false;
	m_v3Min = vector3(0.0f);
	m_v3Max = vector3(0.0f);
	m_v3Centroid = vector3(0.0f);
	m_sName = "NULL";
	m_v3Min2 = vector3(0.0f);
	m_v3Max2 = vector3(0.0f);
}
void BoundingBoxClass::Swap(BoundingBoxClass& other)
{
	std::swap(m_bInitialized, other.m_bInitialized);
	std::swap(m_v3Min, other.m_v3Min);
	std::swap(m_v3Max, other.m_v3Max);
	std::swap(m_v3Centroid, other.m_v3Centroid);
	std::swap(m_sName, other.m_sName);
	std::swap(m_v3Min2, other.m_v3Min2);
	std::swap(m_v3Max2, other.m_v3Max2);
}
void BoundingBoxClass::Release(void)
{
	//No pointers to release
}
//The big 3
BoundingBoxClass::BoundingBoxClass(){Init();}
BoundingBoxClass::BoundingBoxClass(BoundingBoxClass const& other)
{
	m_bInitialized = other.m_bInitialized;
	m_v3Min = other.m_v3Min;
	m_v3Max = other.m_v3Max;
	m_v3Centroid = other.m_v3Centroid;
	m_sName = other.m_sName;
	m_v3Min2 = other.m_v3Min2;
	m_v3Max2 = other.m_v3Max2;
}
BoundingBoxClass& BoundingBoxClass::operator=(BoundingBoxClass const& other)
{
	if(this != &other)
	{
		Release();
		Init();
		BoundingBoxClass temp(other);
		Swap(temp);
	}
	return *this;
}
BoundingBoxClass::~BoundingBoxClass(){Release();};
//Accessors
bool BoundingBoxClass::IsInitialized(void){ return m_bInitialized; }
vector3 BoundingBoxClass::GetMinimumOBB(void){ return m_v3Min; }
vector3 BoundingBoxClass::GetMaximumOBB(void){ return m_v3Max; }
vector3 BoundingBoxClass::GetCentroid(void){ return m_v3Centroid; }
String BoundingBoxClass::GetName(void){return m_sName;}
//Methods
void BoundingBoxClass::GenerateOrientedBoundingBox(String a_sInstanceName)
{
	//If this has already been initialized there is nothing to do here
	if(m_bInitialized)
		return;
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	if(pMeshMngr->IsInstanceCreated(a_sInstanceName))
	{
		m_sName = a_sInstanceName;
		
		std::vector<vector3> lVertices = pMeshMngr->GetVertices(m_sName);
		std::vector<vector3> l2Vertices = pMeshMngr->GetVertices(m_sName);
		unsigned int nVertices = lVertices.size();
		unsigned int OVertices =l2Vertices.size();
		m_v3Centroid = lVertices[0];
		m_v3Max = lVertices[0];
		m_v3Min = lVertices[0];
		m_v3Max2 = l2Vertices[0];
		m_v3Min2 = l2Vertices[0];
		for(unsigned int nVertex = 1; nVertex < nVertices; nVertex++)
		{
				//m_v3Centroid += lVertices[nVertex];
				if(m_v3Min.x > lVertices[nVertex].x)
					m_v3Min.x = lVertices[nVertex].x;
				else if(m_v3Max.x < lVertices[nVertex].x)
					m_v3Max.x = lVertices[nVertex].x;
			
				if(m_v3Min.y > lVertices[nVertex].y)
					m_v3Min.y = lVertices[nVertex].y;
				else if(m_v3Max.y < lVertices[nVertex].y)
					m_v3Max.y = lVertices[nVertex].y;

				if(m_v3Min.z > lVertices[nVertex].z)
					m_v3Min.z = lVertices[nVertex].z;
				else if(m_v3Max.z < lVertices[nVertex].z)
					m_v3Max.z = lVertices[nVertex].z;
			
		}
		m_v3Centroid = (m_v3Min + m_v3Max) / 2.0f;

		m_v3Size.x = glm::distance(vector3(m_v3Min.x, 0.0f, 0.0f), vector3(m_v3Max.x, 0.0f, 0.0f));
		m_v3Size.y = glm::distance(vector3(0.0f, m_v3Min.y, 0.0f), vector3(0.0f, m_v3Max.y, 0.0f));
		m_v3Size.z = glm::distance(vector3(0.0f, 0.0f, m_v3Min.z), vector3(0.0f, 0.0f, m_v3Max.z));

		m_bInitialized = true;
	}
}
void BoundingBoxClass::GenerateAxisAlignedBoundingBox(matrix4 a_m4ModeltoWorld)
{
	//Generate the Axis Aligned Bounding Box here based on the Oriented Bounding Box

	//Points of AABB 
	vector3 PointA(m_v3Min.x, m_v3Max.y, 0.0f);
	vector3 PointB(m_v3Max.x, m_v3Min.y, 0.0f);
	vector3 PointC(m_v3Min.x, m_v3Min.y, 0.0f);
	vector3 PointD(m_v3Max.x, m_v3Max.y, 0.0f);

	if(m_bInitialized)
		return;
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	std::vector<vector3> l2Vertices = pMeshMngr->GetVertices(m_sName);
	unsigned int OVertices =l2Vertices.size();
	m_v3Centroid = l2Vertices[0];
	m_v3Max2 = l2Vertices[0];
	m_v3Min2 = l2Vertices[0];

	for(unsigned int oVertex = 1; oVertex < OVertices; oVertex++)
	{
		//Check PointA against other points
		if(PointA.x > PointB.x && PointA.x > PointC.x && PointA.x > PointD.x)
		{
			m_v3AABBSize.x = PointA.x;
		}
	
		else if(PointB.x > PointA.x && PointB.x > PointC.x && PointB.x > PointD.x)
		{
			m_v3AABBSize.x = PointB.x;
		}

		else if(PointC.x > PointA.x && PointC.x > PointB.x && PointC.x > PointD.x)
		{
			m_v3AABBSize.x = PointC.x;
		}

		else if(PointD.x > PointA.x && PointD.x > PointC.x && PointD.x > PointB.x)
		{
			m_v3AABBSize.x = PointD.x;
		}

		//Check PointB against other points
		if(PointA.y > PointB.y && PointA.y > PointC.y && PointA.y > PointD.y)
		{
			m_v3AABBSize.y = PointA.y;
		}

		else if(PointB.y > PointA.y && PointB.y > PointC.y && PointB.y > PointD.y)
		{
			m_v3AABBSize.y = PointB.y;
		}

		else if(PointC.y > PointA.y && PointC.y > PointB.y && PointC.y > PointD.y)
		{
			m_v3AABBSize.y = PointC.y;
		}

		else if(PointD.y > PointA.y && PointD.y > PointC.y && PointD.y > PointB.y)
		{
			m_v3AABBSize.y = PointD.y;
		}

    }
	
}
void BoundingBoxClass::AddBoxToRenderList(matrix4 a_m4ModelToWorld, vector3 a_vColor, bool a_bRenderCentroid)
{
	if(!m_bInitialized)
		return;
	MeshManagerSingleton* pMeshMngr = MeshManagerSingleton::GetInstance();
	if(a_bRenderCentroid)
		pMeshMngr->AddAxisToQueue(a_m4ModelToWorld * glm::translate(m_v3Centroid));
	pMeshMngr->AddCubeToQueue(a_m4ModelToWorld * glm::translate(m_v3Centroid) * glm::scale(m_v3Size), a_vColor, MERENDER::WIRE);
	pMeshMngr->AddCubeToQueue(a_m4ModelToWorld * glm::translate(m_v3Centroid) * glm::scale(vector3(2.0f)), a_vColor, MERENDER::WIRE);
}