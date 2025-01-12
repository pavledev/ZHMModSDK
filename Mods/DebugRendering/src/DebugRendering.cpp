#include <IconsMaterialDesign.h>

#include "imgui.h"

#include <Glacier/ZGridManager.h>
#include <Glacier/ZHM5GridManager.h>
#include <Glacier/ZColor.h>
#include <Glacier/ZCameraEntity.h>
#include <Glacier/ZPathfinder.h>

#include "DebugRendering.h"
#include <Globals.h>
#include <Functions.h>

void DebugRendering::OnDrawMenu()
{
    if (ImGui::Button(ICON_MD_MONITOR " Debug Rendering"))
    {
        isOpen = !isOpen;
    }
}

void DebugRendering::OnDrawUI(const bool hasFocus)
{
    if (!isOpen)
    {
        return;
    }

	ImGui::PushFont(SDK()->GetImGuiBlackFont());
	auto s_Showing = ImGui::Begin("DEBUG RENDERING", &isOpen);
	ImGui::PushFont(SDK()->GetImGuiRegularFont());

    if (s_Showing)
    {
        if (ImGui::CollapsingHeader("AI Grid"))
        {
            if (ImGui::Checkbox("Render AI Grid", &renderAIGrid))
            {
                if (triangles.size() == 0)
                {
                    GenerateReasoningGridVertices();
                }
            }

            ImGui::Checkbox("Show Visibility", &showVisibility);
            ImGui::Checkbox("Show Layers", &showLayers);
            ImGui::Checkbox("Show Indices", &showIndices);
        }

		if (ImGui::CollapsingHeader("Guide Path Finder"))
		{
			if (ImGui::Checkbox("Render Guide Path Finder", &renderGuidePathFinder))
			{
				if (navMesh.m_areas.size() == 0)
				{
					uintptr_t s_NavpData = reinterpret_cast<uintptr_t>(Globals::Pathfinder->m_NavPowerResources[0].m_pNavpowerResource);
					uint32_t s_NavpDataSize = Globals::Pathfinder->m_NavPowerResources[0].m_nNavpowerResourceSize;
					static const SVector4 lineColor = SVector4(0.f, 1.f, 0.f, 1.f);

					navMesh.read(s_NavpData, s_NavpDataSize);

					vertices.resize(navMesh.m_areas.size());
					indices.resize(navMesh.m_areas.size());
					lines.reserve(navMesh.m_areas.size() * 3);

					for (size_t i = 0; i < navMesh.m_areas.size(); ++i)
					{
						const size_t vertexCount = navMesh.m_areas[i].m_edges.size();

						vertices[i].reserve(vertexCount);

						for (size_t j = 0; j < vertexCount; ++j)
						{
							vertices[i].push_back(navMesh.m_areas[i].m_edges[j]->m_pos);

							static const float zOffset = 0.075f;

							const size_t nextIndex = (j + 1) % vertexCount;
							Line& line = lines.emplace_back();

							line.start = navMesh.m_areas[i].m_edges[j]->m_pos;
							line.startColor = lineColor;
							line.start.z += zOffset;

							line.end = navMesh.m_areas[i].m_edges[nextIndex]->m_pos;
							line.endColor = lineColor;
							line.end.z += zOffset;
						}

						VertexTriangluation(vertices[i], indices[i]);
					}
				}
			}
		}
    }

    ImGui::PopFont();
    ImGui::End();
    ImGui::PopFont();
}

void DebugRendering::OnDraw3D(IRenderer* p_Renderer)
{
	if (p_Renderer->GetCurrentPrimitiveType() == PrimitiveType::Text2D)
	{
		if (renderGuidePathFinder)
		{
			ZPFObstacleManagerDeprecated* obstacleManagerDeprecated = static_cast<ZPFObstacleManagerDeprecated*>(Globals::Pathfinder->m_obstacleManager);

			for (size_t i = 0; i < obstacleManagerDeprecated->m_obstacles.size(); ++i)
			{
				const SMatrix transform = obstacleManagerDeprecated->m_obstacles[i].GetTransform();
				SVector2 s_ScreenPos;
				if (p_Renderer->WorldToScreen(SVector3(transform.Trans.x, transform.Trans.y, transform.Trans.z + 2.05f), s_ScreenPos))
					p_Renderer->DrawText2D("Obstacle", s_ScreenPos, SVector4(1.f, 0.f, 0.f, 1.f), 0.f, 0.5f);
			}
		}
	}
}

void DebugRendering::OnDepthDraw3D(IRenderer* p_Renderer)
{
    if (renderAIGrid)
    {
		RenderReasoningGrid(p_Renderer);
    }

    if (renderGuidePathFinder)
    {
		RenderNavMesh(p_Renderer);
    }
}

SVector3 CalculatePolygonCentroid(const std::vector<SVector3>& vertices)
{
    SVector3 centroid(0.0f, 0.0f, 0.0f);

    for (const auto& vertex : vertices)
    {
        centroid.x += vertex.x;
        centroid.y += vertex.y;
        centroid.z += vertex.z;
    }

    // Divide by the number of vertices to get the average (centroid)
    float numVertices = static_cast<float>(vertices.size());
    centroid.x /= numVertices;
    centroid.y /= numVertices;
    centroid.z /= numVertices;

    return centroid;
}

void DebugRendering::GenerateReasoningGridVertices()
{
    //triangles.clear();
    //lines.clear();

    GenerateVerticesForSmallQuads();
    GenerateVerticesForLargeQuads();
    GenerateVerticesForQuadBorderLines();
    GenerateVerticesForNeighborConnectionLines();
}

void DebugRendering::GenerateVerticesForSmallQuads()
{
	const SReasoningGrid* reasoningGrid = *Globals::ActiveGrid;
    const size_t waypointCount = reasoningGrid->m_WaypointList.size();

	triangles.reserve(waypointCount * 2);

    for (size_t i = 0; i < waypointCount; ++i)
    {
        const float halfBaseLength = 0.1f;
        const float4& vertexPostion = reasoningGrid->m_WaypointList[i].vPos;
        const SVector4 vertexColor = SVector4(0.f, 0.33333f, 1.f, 0.62745f);
        Triangle& triangle1 = triangles.emplace_back();
        Triangle& triangle2 = triangles.emplace_back();

        triangle1.vertexPosition1 = {
            vertexPostion.x - halfBaseLength,
            vertexPostion.y - halfBaseLength,
            vertexPostion.z + halfBaseLength
        };

        triangle1.vertexPosition2 = {
            vertexPostion.x + halfBaseLength,
            vertexPostion.y - halfBaseLength,
            vertexPostion.z + halfBaseLength
        };

        triangle1.vertexPosition3 = {
            vertexPostion.x - halfBaseLength,
            vertexPostion.y + halfBaseLength,
            vertexPostion.z + halfBaseLength
        };

        triangle1.vertexColor1 = vertexColor;
        triangle1.vertexColor2 = vertexColor;
        triangle1.vertexColor3 = vertexColor;

        triangle2.vertexPosition1 = {
            vertexPostion.x + halfBaseLength,
            vertexPostion.y - halfBaseLength,
            vertexPostion.z + halfBaseLength
        };

        triangle2.vertexPosition2 = {
            vertexPostion.x + halfBaseLength,
            vertexPostion.y + halfBaseLength,
            vertexPostion.z + halfBaseLength
        };

        triangle2.vertexPosition3 = {
            vertexPostion.x - halfBaseLength,
            vertexPostion.y + halfBaseLength,
            vertexPostion.z + halfBaseLength
        };

        triangle2.vertexColor1 = vertexColor;
        triangle2.vertexColor2 = vertexColor;
        triangle2.vertexColor3 = vertexColor;
    }
}

void DebugRendering::GenerateVerticesForLargeQuads()
{
	ZGridManager* gridManager = *Globals::GridManager;
	const SReasoningGrid* reasoningGrid = *Globals::ActiveGrid;
    size_t waypointCount = reasoningGrid->m_WaypointList.size();

	triangles.reserve(waypointCount * 2);

    for (uint32_t w = 0; w < waypointCount; ++w)
    {
        const float zOffset = 0.05f;
        float4 vertexPostion = reasoningGrid->m_WaypointList[w].vPos;
        const SVector4 vertexColor = SVector4(0.33333f, 0.f, 1.f, 0.43922f);
        Triangle& triangle1 = triangles.emplace_back();
        Triangle& triangle2 = triangles.emplace_back();

        vertexPostion.z += zOffset;
		vertexPostion = gridManager->GetCellUpperLeft(vertexPostion, reasoningGrid->m_Properties);

        triangle1.vertexPosition1 = {
            vertexPostion.x,
            vertexPostion.y,
            vertexPostion.z
        };

        triangle1.vertexPosition2 = {
            vertexPostion.x + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.y,
            vertexPostion.z
        };

        triangle1.vertexPosition3 = {
            vertexPostion.x + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.y + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.z
        };

        triangle1.vertexColor1 = vertexColor;
        triangle1.vertexColor2 = vertexColor;
        triangle1.vertexColor3 = vertexColor;

        triangle2.vertexPosition1 = {
            vertexPostion.x,
            vertexPostion.y,
            vertexPostion.z
        };

        triangle2.vertexPosition2 = {
            vertexPostion.x + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.y + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.z
        };

        triangle2.vertexPosition3 = {
            vertexPostion.x,
            vertexPostion.y + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.z
        };

        triangle2.vertexColor1 = vertexColor;
        triangle2.vertexColor2 = vertexColor;
        triangle2.vertexColor3 = vertexColor;
    }
}

void DebugRendering::GenerateVerticesForQuadBorderLines()
{
	ZGridManager* gridManager = *Globals::GridManager;
	const SReasoningGrid* reasoningGrid = *Globals::ActiveGrid;
    const size_t waypointCount = reasoningGrid->m_WaypointList.size();

	lines.reserve(waypointCount * 4);

    for (size_t i = 0; i < waypointCount; ++i)
    {
        Line& topBorder = lines.emplace_back();
        Line& rightBorder = lines.emplace_back();
        Line& bottomBorder = lines.emplace_back();
        Line& leftBorder = lines.emplace_back();
        float4 vertexPostion = reasoningGrid->m_WaypointList[i].vPos;
        const SVector4 vertexColor = SVector4(0.f, 0.f, 0.f, 0.43922f);
        const float zOffset = 0.075f;

        vertexPostion.z += zOffset;
		vertexPostion = gridManager->GetCellUpperLeft(vertexPostion, reasoningGrid->m_Properties);

        topBorder.start = {
            vertexPostion.x,
            vertexPostion.y,
            vertexPostion.z
        };

        topBorder.end = {
            vertexPostion.x + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.y,
            vertexPostion.z
        };

        topBorder.startColor = vertexColor;
        topBorder.endColor = vertexColor;

        rightBorder.start = {
            vertexPostion.x + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.y,
            vertexPostion.z
        };

        rightBorder.end = {
            vertexPostion.x + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.y + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.z
        };

        rightBorder.startColor = vertexColor;
        rightBorder.endColor = vertexColor;

        bottomBorder.start = {
            vertexPostion.x + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.y + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.z
        };

        bottomBorder.end = {
            vertexPostion.x,
            vertexPostion.y + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.z
        };

        bottomBorder.startColor = vertexColor;
        bottomBorder.endColor = vertexColor;

        leftBorder.start = {
            vertexPostion.x,
            vertexPostion.y + reasoningGrid->m_Properties.fGridSpacing,
            vertexPostion.z
        };

        leftBorder.end = {
            vertexPostion.x,
            vertexPostion.y,
            vertexPostion.z
        };

        leftBorder.startColor = vertexColor;
        leftBorder.endColor = vertexColor;
    }
}

void DebugRendering::GenerateVerticesForNeighborConnectionLines()
{
	const SReasoningGrid* reasoningGrid = *Globals::ActiveGrid;
    const size_t waypointCount = reasoningGrid->m_WaypointList.size();

	lines.reserve(waypointCount * 4);

    for (size_t i = 0; i < waypointCount; ++i)
    {
        float4 vertexPostion1 = reasoningGrid->m_WaypointList[i].vPos;
        const SVector4 vertexColor = SVector4(0.f, 0.33333f, 1.f, 0.62745f);
        const float zOffset = 0.1f;

        vertexPostion1.z += zOffset;

        short neighborIndex = 0;
        int j = 4;

        while (j != 0)
        {
            if (reasoningGrid->m_WaypointList[i].Neighbors[neighborIndex] != -1)
            {
                Line& line = lines.emplace_back();
                const short neighbor = reasoningGrid->m_WaypointList[i].Neighbors[neighborIndex];
                float4 vertexPostion2 = reasoningGrid->m_WaypointList[neighbor].vPos;

                vertexPostion2.z += zOffset;

                line.start = {
                    vertexPostion1.x,
                    vertexPostion1.y,
                    vertexPostion1.z
                };

                line.end = {
                    vertexPostion2.x,
                    vertexPostion2.y,
                    vertexPostion2.z
                };

                line.startColor = vertexColor;
                line.endColor = vertexColor;
            }

            ++neighborIndex;
            --j;
        }
    }
}

void DebugRendering::RenderReasoningGrid(IRenderer* p_Renderer)
{
	ZGridManager* gridManager = *Globals::GridManager;
	const SReasoningGrid* reasoningGrid = *Globals::ActiveGrid;
    const size_t waypointCount = reasoningGrid->m_WaypointList.size();

	if (p_Renderer->GetCurrentPrimitiveType() == PrimitiveType::Triangle)
	{
		for (size_t i = 0; i < waypointCount * 2; ++i)
		{
			p_Renderer->DrawTriangle3D(triangles[i].vertexPosition1, triangles[i].vertexColor1,
			                           triangles[i].vertexPosition2, triangles[i].vertexColor2,
			                           triangles[i].vertexPosition3, triangles[i].vertexColor3);
		}

		const ZGridNodeRef& hitmanNode = Globals::HM5GridManager->m_HitmanNode;
		const size_t startIndex = waypointCount * 2;

		static const SVector4 selectedNodeVertexColor = SVector4(0.f, 1.f, 1.f, 0.43922f);
		static const SVector4 largeQuadVertexColor = SVector4(0.33333f, 0.f, 1.f, 0.43922f);

		for (size_t i = startIndex; i < triangles.size(); ++i)
		{
			const unsigned short waypointIndex = static_cast<unsigned short>((i - waypointCount * 2) / 2);

			if (reasoningGrid->GetNode(waypointIndex) == hitmanNode.GetNode())
			{
				triangles[i].vertexColor1 = selectedNodeVertexColor;
				triangles[i].vertexColor2 = selectedNodeVertexColor;
				triangles[i].vertexColor3 = selectedNodeVertexColor;
			}
			else
			{
				if (showVisibility)
				{
				    float rating = 0.f;

				    if (hitmanNode.CheckVisibility(waypointIndex, true, false))
				    {
				        rating = 1.f;
				    }
				    else if (hitmanNode.CheckVisibility(waypointIndex, false, false))
				    {
				        rating = 0.5f;
				    }

				    const unsigned int heatMapColor = (gridManager->GetHeatmapColorFromRating(rating) & 0xFFFFFF) + 0x70000000;
				    const SVector4 vertexColor = ZColor::UnpackUnsigned(heatMapColor);

				    triangles[i].vertexColor1 = vertexColor;
				    triangles[i].vertexColor2 = vertexColor;
				    triangles[i].vertexColor3 = vertexColor;
				}
				else if (showLayers)
				{
					const unsigned int layerIndex = static_cast<unsigned int>(reasoningGrid->m_WaypointList[waypointIndex].nLayerIndex);
					const unsigned int color = (layerIndex << 6) | 0xC0000000;
					const SVector4 vertexColor = ZColor::UnpackUnsigned(color);

					triangles[i].vertexColor1 = vertexColor;
					triangles[i].vertexColor2 = vertexColor;
					triangles[i].vertexColor3 = vertexColor;
				}
				else
				{
				    triangles[i].vertexColor1 = largeQuadVertexColor;
				    triangles[i].vertexColor2 = largeQuadVertexColor;
				    triangles[i].vertexColor3 = largeQuadVertexColor;
				}
			}

			p_Renderer->DrawTriangle3D(triangles[i].vertexPosition1, triangles[i].vertexColor1,
			                           triangles[i].vertexPosition2, triangles[i].vertexColor2,
			                           triangles[i].vertexPosition3, triangles[i].vertexColor3);
		}
	}
	else if (p_Renderer->GetCurrentPrimitiveType() == PrimitiveType::Line)
	{
		for (size_t i = 0; i < lines.size(); ++i)
		{
			p_Renderer->DrawLine3D(lines[i].start, lines[i].end, lines[i].startColor, lines[i].endColor);
		}
	}
	else if (p_Renderer->GetCurrentPrimitiveType() == PrimitiveType::Text3D)
	{
		if (showIndices)
		{
			const auto s_CurrentCamera = Functions::GetCurrentCamera->Call();

			if (!s_CurrentCamera)
				return;

			SMatrix worldMatrix = s_CurrentCamera->GetWorldMatrix();
			const size_t waypointCount = reasoningGrid->m_WaypointList.size();

			std::swap(worldMatrix.YAxis, worldMatrix.ZAxis);

			for (size_t i = 0; i < waypointCount; ++i)
			{
				float4 worldPostion = reasoningGrid->m_WaypointList[i].vPos;

				worldPostion.z += 0.5f;
				worldMatrix.Trans = worldPostion;

				std::string text = std::to_string(i);
				SVector4 color = SVector4(0.f, 0.f, 0.f, 1.f);
				float scale = 0.2f;

				p_Renderer->DrawText3D(text, worldMatrix, color, scale);
			}
		}
	}
}

void DebugRendering::RenderNavMesh(IRenderer* p_Renderer)
{
    static const SVector4 greenTriangleColor = SVector4(0.19608f, 0.80392f, 0.19608f, 0.49804f);
    static const SVector4 yellowTriangleColor = SVector4(1.f, 1.f, 0.f, 0.49804f);
    static const SVector4 lineColor = SVector4(0.f, 1.f, 0.f, 1.f);

	if (p_Renderer->GetCurrentPrimitiveType() == PrimitiveType::Triangle)
	{
		/*for (size_t i = 0; i < navMesh.m_areas.size(); ++i)
		{
			const size_t vertexCount = navMesh.m_areas[i].m_edges.size();
			std::vector<SVector3> vertices;
			std::vector<unsigned short> indices;

			vertices.reserve(vertexCount);

			for (size_t j = 0; j < vertexCount; ++j)
			{
				vertices.push_back(navMesh.m_areas[i].m_edges[j]->m_pos);
			}

			VertexTriangluation(vertices, indices);

			if (navMesh.m_areas[i].m_area->m_usageFlags == NavPower::AreaUsageFlags::AREA_FLAT)
			{
				p_Renderer->DrawMesh(vertices, indices, greenTriangleColor);
			}
			else
			{
				p_Renderer->DrawMesh(vertices, indices, yellowTriangleColor);
			}
		}*/

		for (size_t i = 0; i < navMesh.m_areas.size(); ++i)
		{
			if (navMesh.m_areas[i].m_area->m_usageFlags == NavPower::AreaUsageFlags::AREA_FLAT)
			{
				p_Renderer->DrawMesh(vertices[i], indices[i], greenTriangleColor);
			}
			else
			{
				p_Renderer->DrawMesh(vertices[i], indices[i], yellowTriangleColor);
			}
		}
	}
	else if (p_Renderer->GetCurrentPrimitiveType() == PrimitiveType::Line)
	{
		/*for (size_t i = 0; i < navMesh.m_areas.size(); ++i)
		{
			const size_t vertexCount = navMesh.m_areas[i].m_edges.size();
			static const float zOffset = 0.075f;

			for (size_t j = 0; j < vertexCount; ++j)
			{
				const size_t nextIndex = (j + 1) % vertexCount;
				Line line;

				line.start = navMesh.m_areas[i].m_edges[j]->m_pos;
				line.startColor = lineColor;
				line.start.z += zOffset;

				line.end = navMesh.m_areas[i].m_edges[nextIndex]->m_pos;
				line.endColor = lineColor;
				line.end.z += zOffset;

				p_Renderer->DrawLine3D(line.start, line.end, line.startColor, line.endColor);
			}
		}*/

		for (size_t i = 0; i < lines.size(); ++i)
		{
			p_Renderer->DrawLine3D(lines[i].start, lines[i].end, lines[i].startColor, lines[i].endColor);
		}
	}

	if (p_Renderer->GetCurrentPrimitiveType() == PrimitiveType::Triangle)
	{
		if (renderGuidePathFinder)
		{
			ZPFObstacleManagerDeprecated* obstacleManagerDeprecated = static_cast<ZPFObstacleManagerDeprecated*>(Globals::Pathfinder->m_obstacleManager);

			for (size_t i = 0; i < obstacleManagerDeprecated->m_obstacles.size(); ++i)
			{
				bfx::SystemInstance* sys = Globals::NavPowerSystemInstance;
				bfx::ObstacleDat dat = static_cast<const ZPFObstacleManagerDeprecated::ZPFObstacleInternalDep*>(obstacleManagerDeprecated->m_obstacles[i].m_internal.GetTarget())->m_obstacleDef.m_handle.GetObstacleDat();

				if (dat.m_obstacleBehavior != bfx::OBSTACLE_BEHAVIOR_AUTOMATIC)
				{
					int a = 2;
				}
				const SMatrix transform = obstacleManagerDeprecated->m_obstacles[i].GetTransform();
				const float4 halfSize = obstacleManagerDeprecated->m_obstacles[i].GetHalfSize();
				const SVector4 color = SVector4(1.f, 1.f, 0.f, 0.29804f);
				SVector3 minBound = SVector3(
				    -halfSize.x,
				    -halfSize.y,
				    -halfSize.z);

				SVector3 maxBound = SVector3(
				    halfSize.x,
				    halfSize.y,
				    halfSize.z);

				p_Renderer->DrawBoundingQuads(minBound, maxBound, transform, color);
			}
		}
	}
	else if (p_Renderer->GetCurrentPrimitiveType() == PrimitiveType::Line)
	{
		if (renderGuidePathFinder)
		{
			ZPFObstacleManagerDeprecated* obstacleManagerDeprecated = static_cast<ZPFObstacleManagerDeprecated*>(Globals::Pathfinder->m_obstacleManager);

			for (size_t i = 0; i < obstacleManagerDeprecated->m_obstacles.size(); ++i)
			{
				bfx::SystemInstance* sys = Globals::NavPowerSystemInstance;
				bfx::ObstacleDat dat = static_cast<const ZPFObstacleManagerDeprecated::ZPFObstacleInternalDep*>(obstacleManagerDeprecated->m_obstacles[i].m_internal.GetTarget())->m_obstacleDef.m_handle.GetObstacleDat();

				if (dat.m_obstacleBehavior != bfx::OBSTACLE_BEHAVIOR_AUTOMATIC)
				{
					int a = 2;
				}
				const SMatrix transform = obstacleManagerDeprecated->m_obstacles[i].GetTransform();
				const float4 halfSize = obstacleManagerDeprecated->m_obstacles[i].GetHalfSize();
				const SVector4 color = SVector4(1.f, 1.f, 0.f, 1.f);
				SVector3 minBound = SVector3(
				    -halfSize.x,
				    -halfSize.y,
				    -halfSize.z);

				SVector3 maxBound = SVector3(
				    halfSize.x,
				    halfSize.y,
				    halfSize.z);

				p_Renderer->DrawOBB3D(minBound, maxBound, transform, color);
			}
		}
	}
}

SVector3 CrossV3(const SVector3 a, const SVector3 b)
{
	return SVector3(a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x);
}

// Vector3 Magnitude Calculation
float MagnitudeV3(const SVector3 in)
{
	return (sqrtf(powf(in.x, 2) + powf(in.y, 2) + powf(in.z, 2)));
}

// Vector3 DotProduct
float DotV3(const SVector3 a, const SVector3 b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

float AngleBetweenV3(const SVector3 a, const SVector3 b)
{
	float angle = DotV3(a, b);
	angle /= (MagnitudeV3(a) * MagnitudeV3(b));
	return angle = acosf(angle);
}

SVector3 ProjV3(const SVector3 a, const SVector3 b)
{
	SVector3 bn = b / MagnitudeV3(b);
	return bn * DotV3(a, bn);
}

bool SameSide(SVector3 p1, SVector3 p2, SVector3 a, SVector3 b)
{
	SVector3 cp1 = CrossV3(b - a, p1 - a);
	SVector3 cp2 = CrossV3(b - a, p2 - a);

	if (DotV3(cp1, cp2) >= 0)
		return true;
	else
		return false;
}

// Generate a cross produect normal for a triangle
SVector3 GenTriNormal(SVector3 t1, SVector3 t2, SVector3 t3)
{
	SVector3 u = t2 - t1;
	SVector3 v = t3 - t1;

	SVector3 normal = CrossV3(u, v);

	return normal;
}

// Check to see if a Vector3 Point is within a 3 Vector3 Triangle
bool inTriangle(SVector3 point, SVector3 tri1, SVector3 tri2, SVector3 tri3)
{
	// Test to see if it is within an infinite prism that the triangle outlines.
	bool within_tri_prisim = SameSide(point, tri1, tri2, tri3) && SameSide(point, tri2, tri1, tri3)
		&& SameSide(point, tri3, tri1, tri2);

	// If it isn't it will never be on the triangle
	if (!within_tri_prisim)
		return false;

	// Calulate Triangle's Normal
	SVector3 n = GenTriNormal(tri1, tri2, tri3);

	// Project the point onto this normal
	SVector3 proj = ProjV3(point, n);

	// If the distance from the triangle to the point is 0
	//	it lies on the triangle
	if (MagnitudeV3(proj) == 0)
		return true;
	else
		return false;
}

void DebugRendering::VertexTriangluation(const std::vector<SVector3>& vertices, std::vector<unsigned short>& indices)
{
    // If there are 2 or less verts,
    // no triangle can be created,
    // so exit
    if (vertices.size() < 3)
    {
        return;
    }
    // If it is a triangle no need to calculate it
    if (vertices.size() == 3)
    {
        indices.push_back(0);
        indices.push_back(1);
        indices.push_back(2);
        return;
    }

    // Create a list of vertices
    std::vector<SVector3> tVerts = vertices;

    while (true)
    {
        // For every vertex
        for (int i = 0; i < int(tVerts.size()); i++)
        {
            // pPrev = the previous vertex in the list
            SVector3 pPrev;
            if (i == 0)
            {
                pPrev = tVerts[tVerts.size() - 1];
            }
            else
            {
                pPrev = tVerts[i - 1];
            }

            // pCur = the current vertex;
            SVector3 pCur = tVerts[i];

            // pNext = the next vertex in the list
            SVector3 pNext;
            if (i == tVerts.size() - 1)
            {
                pNext = tVerts[0];
            }
            else
            {
                pNext = tVerts[i + 1];
            }

            // Check to see if there are only 3 verts left
            // if so this is the last triangle
            if (tVerts.size() == 3)
            {
                // Create a triangle from pCur, pPrev, pNext
                for (int j = 0; j < int(tVerts.size()); j++)
                {
                    if (vertices[j] == pCur)
                        indices.push_back(j);
                    if (vertices[j] == pPrev)
                        indices.push_back(j);
                    if (vertices[j] == pNext)
                        indices.push_back(j);
                }

                tVerts.clear();
                break;
            }
            if (tVerts.size() == 4)
            {
                // Create a triangle from pCur, pPrev, pNext
                for (int j = 0; j < int(vertices.size()); j++)
                {
                    if (vertices[j] == pCur)
                        indices.push_back(j);
                    if (vertices[j] == pPrev)
                        indices.push_back(j);
                    if (vertices[j] == pNext)
                        indices.push_back(j);
                }

                SVector3 tempVec;
                for (int j = 0; j < int(tVerts.size()); j++)
                {
                    if (tVerts[j] != pCur
                        && tVerts[j] != pPrev
                        && tVerts[j] != pNext)
                    {
                        tempVec = tVerts[j];
                        break;
                    }
                }

                // Create a triangle from pCur, pPrev, pNext
                for (int j = 0; j < int(vertices.size()); j++)
                {
                    if (vertices[j] == pPrev)
                        indices.push_back(j);
                    if (vertices[j] == pNext)
                        indices.push_back(j);
                    if (vertices[j] == tempVec)
                        indices.push_back(j);
                }

                tVerts.clear();
                break;
            }

            // If Vertex is not an interior vertex
            float angle = AngleBetweenV3(pPrev - pCur, pNext - pCur) * (180 / 3.14159265359);
            if (angle <= 0 && angle >= 180)
                continue;

            // If any vertices are within this triangle
            bool inTri = false;
            for (int j = 0; j < int(vertices.size()); j++)
            {
                if (inTriangle(vertices[j], pPrev, pCur, pNext)
                    && vertices[j] != pPrev
                    && vertices[j] != pCur
                    && vertices[j] != pNext)
                {
                    inTri = true;
                    break;
                }
            }
            if (inTri)
                continue;

            // Create a triangle from pCur, pPrev, pNext
            for (int j = 0; j < int(vertices.size()); j++)
            {
                if (vertices[j] == pCur)
                    indices.push_back(j);
                if (vertices[j] == pPrev)
                    indices.push_back(j);
                if (vertices[j] == pNext)
                    indices.push_back(j);
            }

            // Delete pCur from the list
            for (int j = 0; j < int(tVerts.size()); j++)
            {
                if (tVerts[j] == pCur)
                {
                    tVerts.erase(tVerts.begin() + j);
                    break;
                }
            }

            // reset i to the start
            // -1 since loop will add 1 to it
            i = -1;
        }

        // if no triangles were created
        if (indices.size() == 0)
            break;

        // if no more vertices
        if (tVerts.size() == 0)
            break;
    }
}

DEFINE_ZHM_PLUGIN(DebugRendering);
