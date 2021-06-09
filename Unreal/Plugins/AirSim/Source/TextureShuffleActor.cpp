#include "TextureShuffleActor.h"

void ATextureShuffleActor::SwapTexture_Implementation(int tex_id, int component_id, int material_id)
{
    if (SwappableTextures.Num() < 1)
        return;

    if (!MaterialCacheInitialized) {
        TArray<UStaticMeshComponent*> components;
        GetComponents<UStaticMeshComponent>(components);
        NumComponents = components.Num();
        DynamicMaterialInstances.Init(nullptr, components[component_id]->GetNumMaterials());
        MaterialCacheInitialized = true;
    }

    if (NumComponents == 0 || DynamicMaterialInstances.Num() == 0)
        return;

    tex_id %= SwappableTextures.Num();
    component_id %= NumComponents;
    material_id %= DynamicMaterialInstances.Num();

    if (DynamicMaterialInstances[material_id] == nullptr) {
        DynamicMaterialInstances[material_id] = UMaterialInstanceDynamic::Create(DynamicMaterial, this);
        TArray<UStaticMeshComponent*> components;
        GetComponents<UStaticMeshComponent>(components);
        components[component_id]->SetMaterial(material_id, DynamicMaterialInstances[material_id]);
    }

    DynamicMaterialInstances[material_id]->SetTextureParameterValue("TextureParameter", SwappableTextures[tex_id]);
}