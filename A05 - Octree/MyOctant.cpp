#include "MyOctant.h"

using namespace Simplex;

uint MyOctant::m_uOctantCount = 0;
uint MyOctant::m_uMaxLevel = 3;
uint MyOctant::m_uIdealEntityCount = 5;

Simplex::MyOctant::MyOctant(uint a_nMaxLevel, uint a_nIdealEntityCount)
{
	// Init members
	this->Init();

	// Set passed in args
	this->m_uMaxLevel = a_nIdealEntityCount;
	this->m_uIdealEntityCount = a_nIdealEntityCount;

	this->m_uOctantCount = 0;
	this->m_uID = this->m_uOctantCount;

	// This is the root node, so set it
	this->m_pRoot = this;

	// Clear list for population
	this->m_lChild.clear();

	std::vector<vector3> minMaxes;

	for (int i = 0; i < m_pEntityMngr->GetEntityCount(); i++)
	{
		// Get extremes
		MyRigidBody* currRB = m_pEntityMngr->GetEntity(i)->GetRigidBody();
		minMaxes.push_back(currRB->GetMinGlobal());
		minMaxes.push_back(currRB->GetMaxGlobal());
	}

	MyRigidBody tempRB = MyRigidBody(minMaxes);

	// Get the center
	this->m_v3Center = tempRB.GetCenterLocal();

	vector3 halfWidth = tempRB.GetHalfWidth();

	minMaxes.clear();
	// Get largest and use as size
	this->m_fSize = (halfWidth.x > halfWidth.y) ? halfWidth.x : halfWidth.y;
	this->m_fSize = (this->m_fSize >= halfWidth.z) ? this->m_fSize : halfWidth.z;

	// Use fMax for extents
	this->m_v3Min = this->m_v3Center - vector3(this->m_fSize);
	this->m_v3Min = this->m_v3Center + vector3(this->m_fSize);

	// this is now an octant, so increment
	this->m_uOctantCount++;

	// Now construct the tree
	this->ConstructTree(this->m_uMaxLevel);
}

Simplex::MyOctant::MyOctant(vector3 a_v3Center, float a_fSize)
{
	// Init members
	this->Init();

	// Set passed in args
	this->m_v3Center = a_v3Center;
	this->m_fSize = a_fSize;

	// Args can be used to det max and min, calculate
	vector3 extents = vector3(this->m_fSize) / 2.0f;

	this->m_v3Max = this->m_v3Center + extents;
	this->m_v3Min = this->m_v3Center - extents;

	// this is now an octant, so increment
	this->m_uOctantCount++;
}

Simplex::MyOctant::MyOctant(MyOctant const& other)
{
	// Set all their things to mine
	this->m_fSize = other.m_fSize;
	this->m_lChild = other.m_lChild;
	this->m_pParent = other.m_pParent;
	this->m_pRoot = other.m_pRoot;
	this->m_uChildren = other.m_uChildren;
	this->m_uID = other.m_uID;
	this->m_uLevel = other.m_uLevel;
	this->m_v3Center = other.m_v3Center;
	this->m_v3Max = other.m_v3Max;
	this->m_v3Min = other.m_v3Min;

	// These are singletons, get them natively
	this->m_pMeshMngr = MeshManager::GetInstance();
	this->m_pEntityMngr = MyEntityManager::GetInstance();

	// Deeper array copy
	for (uint i = 0; i < 8; i++)
	{
		this->m_pChild[i] = other.m_pChild[i];
	}
}

MyOctant& Simplex::MyOctant::operator=(MyOctant const& other)
{
	if (this != &other) {
		this->Release();
		this->Init();
		MyOctant t(other);
		this->Swap(t);
	}
	return *this;
}

Simplex::MyOctant::~MyOctant(void)
{
	this->Release();
}

void Simplex::MyOctant::Swap(MyOctant& other)
{
	//std::swap(this->, other.);
	std::swap(this->m_fSize, other.m_fSize);
	std::swap(this->m_lChild , other.m_lChild);
	std::swap(this->m_pParent, other.m_pParent);
	std::swap(this->m_pRoot, other.m_pRoot);
	std::swap(this->m_uChildren, other.m_uChildren);
	std::swap(this->m_uID, other.m_uID);
	std::swap(this->m_uLevel, other.m_uLevel);
	std::swap(this->m_v3Center, other.m_v3Center);
	std::swap(this->m_v3Max, other.m_v3Max);
	std::swap(this->m_v3Min, other.m_v3Min);

	for (int i = 0; i < 8; i++)
		std::swap(m_pChild[i], other.m_pChild[i]);

	this->m_pMeshMngr = MeshManager::GetInstance();
	this->m_pEntityMngr = MyEntityManager::GetInstance();
}

float Simplex::MyOctant::GetSize(void)
{
	return this->m_fSize;
}

vector3 Simplex::MyOctant::GetCenterGlobal(void)
{
	return this->m_v3Center;
}

vector3 Simplex::MyOctant::GetMinGlobal(void)
{
	return this->m_v3Min;
}

vector3 Simplex::MyOctant::GetMaxGlobal(void)
{
	return this->m_v3Max;
}

bool Simplex::MyOctant::IsColliding(uint a_uRBIndex)
{
	// TODO: To format
	uint nObjectCount = m_pEntityMngr->GetEntityCount();

	if (a_uRBIndex >= nObjectCount)
		return false;

	MyEntity* pEntity = m_pEntityMngr->GetEntity(a_uRBIndex);
	MyRigidBody* pRigidBody = pEntity->GetRigidBody();
	vector3 v3MaxD = pRigidBody->GetMaxGlobal();
	vector3 v3MinD = pRigidBody->GetMinGlobal();

	if (m_v3Max.x < v3MinD.x)
		return false;
	if (m_v3Min.x > v3MaxD.x)
		return false;

	if (m_v3Max.y < v3MinD.y)
		return false;
	if (m_v3Min.y > v3MaxD.y)
		return false;

	if (m_v3Max.z < v3MinD.z)
		return false;
	if (m_v3Min.z > v3MaxD.z)
		return false;
}

void Simplex::MyOctant::Display(uint a_nIndex, vector3 a_v3Color)
{
	// TODO: To Format
	if (m_uID == a_nIndex)
	{
		m_pMeshMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) * glm::scale(vector3(m_fSize)), a_v3Color, RENDER_WIRE);
		return;
	}

	for (uint nIndex = 0; nIndex < m_uChildren; nIndex++)
	{
		m_pChild[nIndex]->Display(a_nIndex);
	}
}

void Simplex::MyOctant::Display(vector3 a_v3Color)
{
	// TODO: Format
	for (uint nIndex = 0; nIndex < m_uChildren; nIndex++)
	{
		m_pChild[nIndex]->Display(a_v3Color);
	}
	m_pMeshMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) * glm::scale(vector3(m_fSize)), a_v3Color, RENDER_WIRE);
}

void Simplex::MyOctant::DisplayLeafs(vector3 a_v3Color)
{
	// TODO: Format
	uint nLeafs = m_lChild.size();

	for (uint nChild = 0; nChild < nLeafs; nChild++)
	{
		m_lChild[nChild]->Display(a_v3Color);
	}
	m_pMeshMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) * glm::scale(vector3(m_fSize)), a_v3Color, RENDER_WIRE);
}

void Simplex::MyOctant::ClearEntityList(void)
{
	// TODO: Format
	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->ClearEntityList();
	}
	m_EntityList.clear();
}

uint Simplex::MyOctant::GetOctantCount(void)
{
	return this->m_uOctantCount;
}

void Simplex::MyOctant::Release(void)
{
	// If this is the root, clear the branches.
	if (this->m_uLevel == 0)
		this->KillBranches();
	this->m_fSize = 0.0f;
	this->m_uChildren = 0;
	this->m_EntityList.clear();
	this->m_lChild.clear();
}

void Simplex::MyOctant::Init(void)
{
	this->m_fSize = 0.0f;
	this->m_uID = m_uOctantCount;
	this->m_uLevel = 0;
	this->m_v3Center = vector3(0.0f);
	// TODO: Finish
	this->m_uOctantCount = 0;
	this->m_uMaxLevel = 3;
	this->m_uIdealEntityCount = 5;

	this->m_fSize = 0.0f;
	this->m_pMeshMngr = MeshManager::GetInstance();
	this->m_pEntityMngr = MyEntityManager::GetInstance();
	this->m_pParent = nullptr;
	this->m_pRoot = nullptr;
	this->m_uChildren = 0;
	this->m_uID = this->m_uOctantCount;
	this->m_uLevel = 0;
	this->m_v3Center = vector3(0.0f);
	this->m_v3Max = vector3(0.0f);
	this->m_v3Min = vector3(0.0f);
	for (int n = 0; n < 8; n++)
	{
		this->m_pChild[n] = nullptr;
	}
}

void Simplex::MyOctant::ConstructList(void)
{
	// TODO: format
	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->ConstructList();
	}
	if (m_EntityList.size() > 0)
	{
		m_pRoot->m_lChild.push_back(this);
	}
}

void Simplex::MyOctant::Subdivide(void)
{
	// TODO: Format
	printf("%d %d\n", m_uLevel, m_uMaxLevel);

	if (m_uLevel >= m_uMaxLevel)
		return;
	if (m_uChildren != 0)
		return;

	m_uChildren = 8;

	float fSize = m_fSize / 4.0f;
	float fSizeD = fSize * 2.0f;
	vector3 v3Center;

	v3Center = m_v3Center;
	v3Center.x -= fSize;
	v3Center.y -= fSize;
	v3Center.z -= fSize;
	m_pChild[0] = new MyOctant(v3Center, fSizeD);

	v3Center.x += fSizeD;
	m_pChild[1] = new MyOctant(v3Center, fSizeD);

	v3Center.z += fSizeD;
	m_pChild[2] = new MyOctant(v3Center, fSizeD);

	v3Center.x -= fSizeD;
	m_pChild[3] = new MyOctant(v3Center, fSizeD);

	v3Center.y += fSizeD;
	m_pChild[4] = new MyOctant(v3Center, fSizeD);

	v3Center.z -= fSizeD;
	m_pChild[5] = new MyOctant(v3Center, fSizeD);

	v3Center.x += fSizeD;
	m_pChild[6] = new MyOctant(v3Center, fSizeD);

	v3Center.z += fSizeD;
	m_pChild[7] = new MyOctant(v3Center, fSizeD);

	for (uint nIndex = 0; nIndex < 8; nIndex++)
	{
		m_pChild[nIndex]->m_pRoot = m_pRoot;
		m_pChild[nIndex]->m_pParent = this;
		m_pChild[nIndex]->m_uLevel = m_uLevel + 1;
		if (m_pChild[nIndex]->ContainsMoreThan(m_uIdealEntityCount))
		{
			m_pChild[nIndex]->Subdivide();
		}
	}
}

MyOctant* Simplex::MyOctant::GetChild(uint a_nChild)
{
	return this->m_pChild[a_nChild];
}

MyOctant* Simplex::MyOctant::GetParent(void)
{
	return this->m_pParent;
}

bool Simplex::MyOctant::IsLeaf(void)
{
	return this->m_uChildren == 0;
}

bool Simplex::MyOctant::ContainsMoreThan(uint a_nEntities)
{
	// TODO: format
	uint nCount = 0;
	uint nObjectCount = m_pEntityMngr->GetEntityCount();

	for (uint n = 0; n < nObjectCount; n++)
	{
		if (IsColliding(n))
			nCount++;
		if (nCount > a_nEntities)
			return true;
	}
	return false;
}

void Simplex::MyOctant::KillBranches(void)
{
	// TODO: Format
	for (uint nIndex = 0; nIndex < m_uChildren; nIndex++)
	{
		m_pChild[nIndex]->KillBranches();
		delete m_pChild[nIndex];
		m_pChild[nIndex] = nullptr;
	}
	m_uChildren = 0;
}

void Simplex::MyOctant::ConstructTree(uint a_nMaxLevel)
{
	if (m_uLevel != 0)
		return;

	m_uMaxLevel = a_nMaxLevel;
	m_uOctantCount = 1;
	m_EntityList.clear();
	KillBranches();
	m_lChild.clear();

	if (ContainsMoreThan(m_uIdealEntityCount))
	{
		this->Subdivide();
	}
	this->AssignIDtoEntity();
	this->ConstructList();
}

void Simplex::MyOctant::AssignIDtoEntity()
{
	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->AssignIDtoEntity();
	}
	if (m_uChildren == 0)
	{
		uint nEntities = m_pEntityMngr->GetEntityCount();
		for (uint nIndex = 0; nIndex < nEntities; nIndex++)
		{
			if (IsColliding(nIndex))
			{
				m_EntityList.push_back(nIndex);
				m_pEntityMngr->AddDimension(nIndex, m_uID);
			}
		}
	}
}
