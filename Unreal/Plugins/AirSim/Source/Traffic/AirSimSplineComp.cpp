// Fill out your copyright notice in the Description page of Project Settings.

#include "AirSimSplineComp.h"

void UAirSimSplineComp::insertMetaData(FAirSimSplinePointMetaData MetaData, int32 Index)
{
	spline_metadata_.Insert(MetaData, Index);
}
void UAirSimSplineComp::generateMetaData()
{
	FInterpCurveVector& SplinePosition = GetSplinePointsPosition();

	spline_metadata_.Empty(0);
	spline_metadata_.AddDefaulted(SplinePosition.Points.Num());
}
void UAirSimSplineComp::setNewLastSelectedIndex(int32 NewIndex)
{
	last_selected_index = NewIndex;
}