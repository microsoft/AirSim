# Camera Views

The camera views that are shown on screen are the camera views you can fetch via the DroneControllerBase::getImageTypeForCamera API.
Note that getImageTypeForCamera returns .png compressed images.

![Cameras](images/cameras.png)

From left to right is the depth view, segmentation view and the FPV view.  These views must be visible in order for
getImageTypeForCamera to work.

## Depth View

The depth view provides a ground truth depth map image.  This depth is computed from the 3D model information and therefore can
be used to test any other algorithm that tries to compute depth from the images.

## Segmentation View

This view randomly colors objects in the scene based on information on how the 3D model was built.  These colors are currently
random, so they contain no semantic tagging.  For example, in the above picture the hedge in the forground is made up of separate
hedge segments so each segment gets a different color as a result, even though it is the same type of hedge.

If you want your Unreal environment to provide colors that are meaningful, then override FlyingPawn::setupStencilIDs() function or 
assign proper values in CustomDepthStencilValue property in object browser window in Unreal Editor, then remove the random 
assignment that is happening in AFlyingPawn::setStencilIDs.

## FPV View

This view is simulating a fixed forward pointing camera on the drone.  Note however, that this camera switchs places with the
main game view if the user presses '[' or ']'.  
