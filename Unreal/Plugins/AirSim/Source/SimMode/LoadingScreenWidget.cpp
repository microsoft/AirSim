// Fill out your copyright notice in the Description page of Project Settings.

//#pragma once
//#include "LoadingScreenWidget.h"
//#include "Runtime/ImageWrapper/Public/IImageWrapperModule.h"
//#include "Runtime/ImageWrapper/Public/IImageWrapper.h"
//#include <string>

//UImage ULoadingScreenWidget::setImage()
//{
//	IImageWrapperModule& image_wrapper = FModuleManager::LoadModuleChecked<IIMageWrapperModule>(FName("ImageWrapper"));
//	IImageWrapperPtr ImageWrapper = image_wrapper.CreateImageWrapper(EImageFormat::JPEG);
//
//	TArray< uint8 > raw_file_data;
//	std::string image_file = std::to_string(count) + ".jpg";
//	if (FFileHelper::LoadFileToArray(raw_file_data, image_file.c_str()))
//	{
//		if (ImageWrapper.IsValid() && ImageWrapper->SetCompressed(raw_file_data.GetData(), raw_file_data.Num()))
//		{
//			const TArray<uint8>* UncompressedBGRA = NULL;
//			if (ImageWrapper->GetRaw(ERGBFormat::BGRA, 8, UncompressedBGRA))
//			{
//				mytex = UTexture2D::CreateTransient(ImageWrapper->GetWidth(), ImageWrapper->GetHeight(), PF_B8G8R8A8);
//			}
//		}
//	}
//
//	++count % num_images;
//}

