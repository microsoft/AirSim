#include "AirSimCharacter.h"


void AAirSimCharacter::setFaceExpression(const std::string& expression_name, float value)
{
    unused(expression_name);
    unused(value);
    //derived class should override this
}
float AAirSimCharacter::getFaceExpression(const std::string& expression_name) const
{
    unused(expression_name);
    //derived class should override this

    return common_utils::Utils::nan<float>();
}
std::vector<std::string> AAirSimCharacter::getAvailableFaceExpressions() const
{
    //derived class should override this
    return std::vector<std::string>();
}
void AAirSimCharacter::setSkinDarkness(float value)
{
    unused(value);
    //derived class should override this
}
float AAirSimCharacter::getSkinDarkness() const 
{
    //derived class should override this
    return common_utils::Utils::nan<float>();
}
void AAirSimCharacter::setSkinAgeing(float value)
{
    unused(value);
    //derived class should override this
}
float AAirSimCharacter::getSkinAgeing() const
{
    //derived class should override this
    return common_utils::Utils::nan<float>();
}
void AAirSimCharacter::setHeadRotation(const msr::airlib::Quaternionr& q)
{
    unused(q);
    //derived class should override this
}
msr::airlib::Quaternionr AAirSimCharacter::getHeadRotation() const
{
    //derived class should override this
    return msr::airlib::Quaternionr::Identity();
}
void AAirSimCharacter::setBonePose(const std::string& bone_name, const msr::airlib::Pose& pose)
{
    unused(bone_name);
    unused(pose);
    //derived class should override this
}
msr::airlib::Pose AAirSimCharacter::getBonePose(const std::string& bone_name) const
{
    unused(bone_name);
    //derived class should override this
    return msr::airlib::Pose::nanPose();
}
void AAirSimCharacter::resetBonePose(const std::string& bone_name)
{
    unused(bone_name);
    //derived class should override this
}
