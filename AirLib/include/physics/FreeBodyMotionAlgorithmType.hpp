#ifndef air_freebodymotionalgorithmtype_hpp
#define air_freebodymotionalgorithmtype_hpp

namespace msr { namespace airlib {

	enum FreeBodyMotionAlgorithmType
	{
		/*Object acts as a projectile in free space.*/
		MOTIONTYPE_PROJECTILE,

		/*Object acts as an automobile.*/
		MOTIONTYPE_AUTOMOBILE
	};

}}

#endif