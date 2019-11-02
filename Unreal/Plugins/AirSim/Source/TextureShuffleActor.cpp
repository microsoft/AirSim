#include "TextureShuffleActor.h"

void ATextureShuffleActor::SwapTexture_Implementation(int tex_id)
{
	if (SwappableTextures.Num() < 1)
		return;
	tex_id %= SwappableTextures.Num();

	if (DynamicMaterialInstance == nullptr)
	{
		DynamicMaterialInstance = UMaterialInstanceDynamic::Create(DynamicMaterial, this);
		TArray<UStaticMeshComponent*> components;
		GetComponents<UStaticMeshComponent>(components);
		if (components.Num() < 1)
			return;
		components[0]->SetMaterial(0, DynamicMaterialInstance);
	}

	DynamicMaterialInstance->SetTextureParameterValue("TextureParameter", SwappableTextures[tex_id]);
	Current_Id = tex_id;
}