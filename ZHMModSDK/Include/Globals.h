#pragma once

#include "Common.h"
#include <cstdint>

class ZCollisionManager;
class ZResourceContainer;
class ZResourceManager;
class ZGameStatsManager;
class ZEntityManager;
class ZMemoryManager;
class ZGameLoopManager;
class ZTypeRegistry;
class ZGameTimeManager;
class ZHitman5Module;
class ZGameContext;
class ZActorManager;
class ZRenderManager;
class ZApplicationEngineWin32;
class ZGameUIManager;
class ZCameraManager;
class ZPlayerRegistry;
class ZHM5InputManager;
class ZContentKitManager;
class ZHM5ActionManager;
class ZBehaviorService;
class SPrimitiveBufferData;
class IGameMode;
class IEngineMode;
class ZInputActionManager;
class ZSelectionForFreeCameraEditorStyleEntity;
template <typename T> class TArray;
template <typename T> class TEntityRef;
struct SReasoningGrid;
class ZGridManager;
class ZHM5GridManager;
class ZPathfinder;

namespace bfx
{
    class SystemInstance;
}

class ZHMSDK_API Globals {
public:
    static ZGameLoopManager* GameLoopManager;
    static ZTypeRegistry** TypeRegistry;
    static ZGameTimeManager* GameTimeManager;
    static ZHitman5Module* Hitman5Module;
    static ZGameContext* GameContext;
    static ZActorManager* ActorManager;
    static uint16_t* NextActorId;
    static ZMemoryManager** MemoryManager;
    static ZRenderManager* RenderManager;
    static ZApplicationEngineWin32** ApplicationEngineWin32;
    static ZGameUIManager* GameUIManager;
    static ZEntityManager* EntityManager;
    static ZGameStatsManager* GameStatsManager;
    static ZCameraManager* CameraManager;
    static ZPlayerRegistry* PlayerRegistry;
    static ZHM5InputManager* InputManager;
    static ZResourceContainer** ResourceContainer;
    static ZResourceManager* ResourceManager;
    static ZCollisionManager** CollisionManager;
    static ZContentKitManager* ContentKitManager;
    static ZHM5ActionManager* HM5ActionManager;
    static ZBehaviorService* BehaviorService;
    static ZGridManager** GridManager;
    static ZHM5GridManager* HM5GridManager;
    static ZPathfinder* Pathfinder;
    static bfx::SystemInstance* NavPowerSystemInstance;
    static SPrimitiveBufferData* PrimitiveBufferData;
    static IGameMode** GameMode;
    static IEngineMode** EngineMode;
    static void* ZTemplateEntityBlueprintFactory_vtbl;
    static ZInputActionManager* InputActionManager;
    static int* InputActionManager_BindMem;
    static int* InputActionManager_Seq;
    static TArray<TEntityRef<ZSelectionForFreeCameraEditorStyleEntity>>* Selections;
    static SReasoningGrid** ActiveGrid;
};
