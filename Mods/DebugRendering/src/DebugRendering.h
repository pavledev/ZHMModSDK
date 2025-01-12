#pragma once

#include "IPluginInterface.h"
#include <NavPower.h>

struct SGProperties;

class DebugRendering : public IPluginInterface
{
public:
	void OnDrawMenu() override;
	void OnDrawUI(const bool hasFocus) override;
	void OnDraw3D(IRenderer* p_Renderer) override;
	void OnDepthDraw3D(IRenderer* p_Renderer) override;
	//void OnDepthDraw3D() override;
	//void OnDepthDrawText3D() override;

private:
	void GenerateReasoningGridVertices();
	void GenerateVerticesForSmallQuads();
	void GenerateVerticesForLargeQuads();
	void GenerateVerticesForQuadBorderLines();
	void GenerateVerticesForNeighborConnectionLines();
	void RenderReasoningGrid(IRenderer* p_Renderer);

	void RenderNavMesh(IRenderer* p_Renderer);
	void VertexTriangluation(const std::vector<SVector3>& vertices, std::vector<unsigned short>& indices);

	bool isOpen;

	bool renderAIGrid = false;
	bool showVisibility = false;
	bool showLayers = false;
	bool showIndices = false;

	bool renderGuidePathFinder = false;
	NavPower::NavMesh navMesh;
	std::vector<std::vector<SVector3>> vertices;
	std::vector<std::vector<unsigned short>> indices;

	std::vector<Line> lines;
	std::vector<Triangle> triangles;
};

DECLARE_ZHM_PLUGIN(DebugRendering)
