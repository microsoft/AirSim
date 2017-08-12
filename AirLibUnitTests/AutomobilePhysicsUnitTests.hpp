#ifndef msr_AirLibUnitTests_AutomobilePhysicsUnitTests_hpp
#define msr_AirLibUnitTests_AutomobilePhysicsUnitTests_hpp

#include "automobile\Automobile.hpp"
#include "automobile\AutomobileSedan.hpp"
#include "automobile\AutomobileStaticParams.hpp"
#include "automobile\PneumaticWheel.hpp"
#include "automobile\MagicFormulaWheel.hpp"
#include "automobile\TirFileWheelParameters.hpp"
#include "automobile\BrushWheelParameters.hpp"
#include "automobile\BrushWheel.hpp"
#include "automobile\WheelParameters.hpp"
#include "automobile/AutomobileEngine.hpp"
#include "automobile/AutomobileHardCodedEngine.hpp"
#include "automobile/AutomobileEngineParameters.hpp"
#include "automobile/AutomobileBrake.hpp"
#include "automobile/AutomobileHardCodedBrake.hpp"
#include "physics/World.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "common/DebugClock.hpp"
#include "common/common_utils/Utils.hpp"


#include <cassert>
#include <vector>
#include <fstream>

namespace msr {
	namespace airlib {
		class AutomobilePhysicsUnitTests
		{
		public:
			void RunAll()
			{
				/*TODO:
				 *
				 * Value checks could be a bit more exact, rather than order-of-magnitude and direction.
				 *
				 * Also, verify that the coordinate system specified in car (implicit in the SteeringAngleTurnsAutomobile test) 
				 *    aligns with the Unreal coordinate system
				 * 
				 */

				#if defined(PHYSICS_VERBOSE)
					this->GenerateWheelTestData();
				#endif
				
				this->NoControlSignalAutomobileDoesNotMove();
				this->AccelerationMovesAutomobile();
				this->BrakeSlowsDownAutomobile();
				this->SteeringAngleTurnsAutomobile(); 
			}

			void GenerateWheelTestData()
			{
				ResetClock();

				TestWorld world;


				BrushWheel* bw = static_cast<BrushWheel*>(world.Wheels[0]);

				std::vector<std::vector<real_T>> data = bw->TestForces();

				std::ofstream output_file;
				output_file.open("F:/mechanics/longTire.tsv");
				output_file << "Fx\tk" << std::endl;

				for (size_t i = 0; i < data[0].size(); i++)
				{
					output_file << data[0][i] << "\t" << data[1][i] << std::endl;
				}
				output_file.flush();
				output_file.close();

				output_file.open("F:/mechanics/latTire.tsv");
				output_file << "Fy\ta" << std::endl;

				for (size_t i = 0; i < data[2].size(); i++)
				{
					output_file << data[2][i] << "\t" << data[3][i] << std::endl;
				}
				output_file.flush();
				output_file.close();
			}

			void NoControlSignalAutomobileDoesNotMove()
			{
				ResetClock();

				TestWorld world;

				/*Get current state of the car*/
				Kinematics before(world.Automobile->getKinematics());
				Kinematics::State beforeState = before.getState();

				/*Set the control signals to zero. This should correspond to a car not driving at all.*/
				world.Automobile->SetControlSignals(static_cast<real_T>(0), static_cast<real_T>(0), static_cast<real_T>(1));

				/*Run the car for one second*/
				RunForTime(20E6, 1E9, &world);

				/*Get the state of the car after time has passed*/
				Kinematics after(world.Automobile->getKinematics());
				Kinematics::State afterState = after.getState();

				assert(beforeState.pose.orientation.isApprox(afterState.pose.orientation));
				assert(beforeState.pose.position.isApprox(afterState.pose.position));

				assert(beforeState.twist.angular.isApprox(afterState.twist.angular));
				assert(beforeState.twist.linear.isApprox(afterState.twist.linear));

				assert(beforeState.accelerations.angular.isApprox(afterState.accelerations.angular));
				assert(beforeState.accelerations.linear.isApprox(afterState.accelerations.linear));
				assert(false);
			}

			void AccelerationMovesAutomobile()
			{
				ResetClock();

				TestWorld world;

				/*Get current state of the car*/
				Kinematics before(world.Automobile->getKinematics());
				Kinematics::State beforeState = before.getState();

				/*Set the control signals to accelerate forward.*/
				world.Automobile->SetControlSignals(static_cast<real_T>(0), static_cast<real_T>(1), static_cast<real_T>(0));

				/*Run the car for two seconds*/
				RunForTime(20E6, 2E9, &world);

				/*Get the state of the car after time has passed*/
				Kinematics after(world.Automobile->getKinematics());
				Kinematics::State afterState = after.getState();

				/*Assert that the car has travelled straight ahead*/
				assert(Utils::isDefinitelyGreaterThan(afterState.pose.position.x(), static_cast<real_T>(0)));
				assert(Utils::isApproximatelyZero(afterState.pose.position.y()));
				assert(Utils::isApproximatelyZero(afterState.pose.position.z()));
				assert(afterState.pose.orientation.isApprox(beforeState.pose.orientation));

				/*Velocity should be straight ahead*/
				assert(Utils::isDefinitelyGreaterThan(afterState.twist.linear.x(), static_cast<real_T>(0)));
				assert(Utils::isApproximatelyZero(afterState.twist.linear.y()));
				assert(Utils::isApproximatelyZero(afterState.twist.linear.z()));
				assert(afterState.twist.angular.isApprox(beforeState.twist.angular));

				/*Acceleation should be ahead*/
				assert(Utils::isDefinitelyGreaterThan(afterState.accelerations.linear.x(), static_cast<real_T>(0)));
				assert(Utils::isApproximatelyZero(afterState.accelerations.linear.y()));
				assert(Utils::isApproximatelyZero(afterState.accelerations.linear.z()));
				assert(afterState.accelerations.angular.isApprox(beforeState.accelerations.angular));
			}

			void BrakeSlowsDownAutomobile()
			{
				ResetClock();

				TestWorld worldNoBrake;
				TestWorld worldBrake;

				vector<TestWorld*> worlds;
				worlds.push_back(&worldNoBrake);
				worlds.push_back(&worldBrake);

				/*Set the control signals to accelerate forward.*/
				worldNoBrake.Automobile->SetControlSignals(static_cast<real_T>(0), static_cast<real_T>(1), static_cast<real_T>(0));
				worldBrake.Automobile->SetControlSignals(static_cast<real_T>(0), static_cast<real_T>(1), static_cast<real_T>(0));

				/*Run the cars for two seconds*/
				RunForTime(20E6, 2E9, worlds);

				/*Apply brake to one of the cars. Allow the other one to coast.*/
				worldNoBrake.Automobile->SetControlSignals(static_cast<real_T>(0), static_cast<real_T>(0), static_cast<real_T>(0));
				worldBrake.Automobile->SetControlSignals(static_cast<real_T>(0), static_cast<real_T>(0), static_cast<real_T>(1));

				/*Run the cars for two seconds*/
				RunForTime(20E6, 4E9, worlds);

				/*Get the state of the car after time has passed*/
				Kinematics noBrakeKinematics(worldNoBrake.Automobile->getKinematics());
				Kinematics::State noBrakeState = noBrakeKinematics.getState();

				Kinematics brakeKinematics(worldBrake.Automobile->getKinematics());
				Kinematics::State brakeState = brakeKinematics.getState();

				/*Check that the braking car did not go as far and is moving slower than the car that had no brake applied*/
				assert(Utils::isDefinitelyGreaterThan(noBrakeState.pose.position.x(), brakeState.pose.position.x()));
				assert(Utils::isDefinitelyGreaterThan(noBrakeState.twist.linear.x(), brakeState.twist.linear.x()));
				assert(Utils::isDefinitelyGreaterThan(noBrakeState.accelerations.linear.x(), brakeState.accelerations.linear.x()));

				/*Also check that the braking did not cause the car to turn*/
				assert(noBrakeState.pose.orientation.isApprox(brakeState.pose.orientation));
				assert(noBrakeState.twist.angular.isApprox(brakeState.twist.angular));
				assert(noBrakeState.accelerations.angular.isApprox(brakeState.accelerations.angular));
			}

			void SteeringAngleTurnsAutomobile()
			{
				ResetClock();

				TestWorld leftTurnWorld;
				TestWorld noTurnWorld;
				TestWorld rightTurnWorld;

				vector<TestWorld*> worlds;
				worlds.push_back(&leftTurnWorld);
				worlds.push_back(&noTurnWorld);
				worlds.push_back(&rightTurnWorld);

				/*Set the control signals*/
				leftTurnWorld.Automobile->SetControlSignals(static_cast<real_T>(-M_PIf/8.0f), static_cast<real_T>(0.05), static_cast<real_T>(0));
				noTurnWorld.Automobile->SetControlSignals(static_cast<real_T>(0), static_cast<real_T>(0.05), static_cast<real_T>(0));
				rightTurnWorld.Automobile->SetControlSignals(static_cast<real_T>(M_PIf/8.0f), static_cast<real_T>(0.05), static_cast<real_T>(0));

				/*Open debug streams*/
				std::ofstream leftOut, centerOut, rightOut, otherOut;
				
				#if defined(PHYSICS_VERBOSE)
					leftOut.open("F:/mechanics/left.tsv");
					centerOut.open("F:/mechanics/center.tsv");
					rightOut.open("F:/mechanics/right.tsv");
					otherOut.open("F:/mechanics/other.tsv");

					leftOut << "x\ty\tz" << std::endl;
					centerOut << "x\ty\tz" << std::endl;
					rightOut << "x\ty\tz" << std::endl;
					otherOut << "v\ta" << std::endl;
				#endif
			
				/*Run the cars for 60 seconds*/
				real_T stopTime = 20E6;
				while (stopTime <= 6E10)
				{
					RunForTime(20E6, stopTime, worlds);
					stopTime += 20E6;

					#if defined(PHYSICS_VERBOSE)
						Kinematics leftTurnKinematics = leftTurnWorld.Automobile->getKinematics();
						Kinematics::State leftTurnState = leftTurnKinematics.getState();

						Kinematics noTurnKinematics = noTurnWorld.Automobile->getKinematics();
						Kinematics::State noTurnState = noTurnKinematics.getState();

						Kinematics rightTurnKinematics = rightTurnWorld.Automobile->getKinematics();
						Kinematics::State rightTurnState = rightTurnKinematics.getState();

						leftOut << leftTurnState.pose.position.x() << '\t' << leftTurnState.pose.position.y() << '\t' << leftTurnState.pose.position.z() << std::endl;
						centerOut << noTurnState.pose.position.x() << '\t' << noTurnState.pose.position.y() << '\t' << noTurnState.pose.position.z() << std::endl;
						rightOut << rightTurnState.pose.position.x() << '\t' << rightTurnState.pose.position.y() << '\t' << rightTurnState.pose.position.z() << std::endl;
						otherOut << leftTurnState.twist.linear.norm() << '\t' << leftTurnState.accelerations.linear.norm() * Utils::sgn(leftTurnState.accelerations.linear.x()) << std::endl;
					#endif				
				}

				#if defined(PHYSICS_VERBOSE)
					leftOut.flush();
					leftOut.close();
					centerOut.flush();
					centerOut.close();
					rightOut.flush();
					rightOut.close();
					otherOut.flush();
					otherOut.close();
				#endif
			

				Kinematics leftTurnKinematics = leftTurnWorld.Automobile->getKinematics();
				Kinematics::State leftTurnState = leftTurnKinematics.getState();

				Kinematics noTurnKinematics = noTurnWorld.Automobile->getKinematics();
				Kinematics::State noTurnState = noTurnKinematics.getState();

				Kinematics rightTurnKinematics = rightTurnWorld.Automobile->getKinematics();
				Kinematics::State rightTurnState = rightTurnKinematics.getState();

				/*Assert that the left turn has turned left, and that the right turn has turned right*/
				assert(Utils::isDefinitelyLessThan(leftTurnState.pose.position.y(), static_cast<real_T>(0)));
				assert(Utils::isDefinitelyGreaterThan(rightTurnState.pose.position.y(), static_cast<real_T>(0)));
				
				auto leftTurnMatrix = leftTurnState.pose.orientation.toRotationMatrix();
				auto rightTurnMatrix = rightTurnState.pose.orientation.toRotationMatrix();

				real_T leftTurnAngle = atan2(leftTurnMatrix(1, 0), leftTurnMatrix(0, 0));
				real_T rightTurnAngle = atan2(rightTurnMatrix(1, 0), rightTurnMatrix(0, 0));

				assert(Utils::isDefinitelyLessThan(leftTurnAngle, static_cast<real_T>(0)));
				assert(Utils::isDefinitelyGreaterThan(rightTurnAngle, static_cast<real_T>(0)));

				/*Car motion is symmetric, so the car should turn left as far as it turns right*/
				assert(Utils::isApproximatelyZero(leftTurnState.pose.position.y() + rightTurnState.pose.position.y()));
				assert(Utils::isApproximatelyZero(leftTurnAngle + rightTurnAngle));

				/*Y velocities should be opposite and symmetric*/
				assert(Utils::isDefinitelyLessThan(leftTurnState.twist.linear.y(), static_cast<real_T>(0)));
				assert(Utils::isDefinitelyGreaterThan(rightTurnState.twist.linear.y(), static_cast<real_T>(0)));
				assert(Utils::isDefinitelyLessThan(leftTurnState.twist.angular.z(), static_cast<real_T>(0)));
				assert(Utils::isDefinitelyGreaterThan(rightTurnState.twist.angular.z(), static_cast<real_T>(0)));

				assert(Utils::isApproximatelyZero(leftTurnState.twist.linear.y() + rightTurnState.twist.linear.y()));
				assert(Utils::isApproximatelyZero(leftTurnState.twist.angular.z() + rightTurnState.twist.angular.z()));

			}

		private:
			class TestWorld
			{
			public:
				AutomobileStaticParams *StaticParams;
				std::vector<PneumaticWheel*> Wheels;
				PhysicsEngineBase *PhysicsEngine;
				Environment *Environment;
				Automobile *Automobile;
				AutomobileEngine *AutomobileEngine;
				AutomobileEngineParameters *AEngineParameters;
				AutomobileBrake *Brake;
				AutomobileBrakeParameters *BrakeParameters;

				TestWorld()
				{
					/*Create the automobile*/
					this->StaticParams = new AutomobileStaticParams();
					this->StaticParams->MaxSteeringAngleRadians = M_PIf / 4.0f;
					this->StaticParams->MaximumBrakeTorque = 2000.0f;
					this->StaticParams->BrakeTorquePerRadPerSecond = 50.0f;
					this->StaticParams->MaximumThrottleTorque = 150.0f;
					this->StaticParams->Mass = 900.0f;
					this->StaticParams->LateralWheelBase = 2.0f;
					this->StaticParams->LongitudinalWheelBase = 4.0f;
					this->StaticParams->DragCoefficient = 0.3f;
					this->StaticParams->CrossSectionalArea = 2.2f;

					BrushWheelParameters wheelParameters("..\\x64\\Debug\\include\\test.brush");
					
					this->Wheels.clear();
					for (int i = 0; i < 4; i++)
					{
						this->Wheels.push_back(new BrushWheel(wheelParameters, true, true, (i < 2)));
					}

					this->BrakeParameters = new AutomobileBrakeParameters(150);
					this->Brake = new AutomobileHardCodedBrake(this->BrakeParameters);

					/*Stall -> 1mph at 26" diameter wheels*/
					/*RedLine -> 80mph at 26" diameter wheels*/
					this->AEngineParameters = new AutomobileEngineParameters(1.37f, 109.5f, 325.0f);
					this->AutomobileEngine = new AutomobileHardcodedEngine(this->AEngineParameters);

					auto initialKinematics = Kinematics::State::zero();
					initialKinematics.pose = Pose::zero();
					Environment::State initialEnvironment;
					initialEnvironment.position = initialKinematics.pose.position;
					this->Environment = new msr::airlib::Environment(initialEnvironment);

					Matrix3x3r inertia = Matrix3x3r::Zero();
					//inertia(0, 0) = 75.0f * 1.21f * 1.21f;
					//inertia(1, 1) = 75.0f * 4.87f * 1.21f;
					//inertia(2, 2) = 75.0f * 1.21f * 4.87f;

					inertia(0, 0) = 5000.0f;
					inertia(1, 1) = 5000.0f;
					inertia(2, 2) = 5000.0f;

					this->Automobile = new AutomobileSedan(inertia, initialKinematics, this->Environment, this->Wheels, this->AutomobileEngine, this->Brake, *(this->StaticParams));

					this->PhysicsEngine = new FastPhysicsEngine();
					this->PhysicsEngine->insert(this->Automobile);
				}

				~TestWorld()
				{
					if (this->Automobile != nullptr)
					{
						delete this->Automobile;
					}

					if (this->StaticParams != nullptr)
					{
						delete this->StaticParams;
					}

					if (this->Wheels.size ()> 0)
					{
						for (unsigned int i = 0; i < this->Wheels.size(); i++)
						{
							delete this->Wheels[i];
						}
						this->Wheels.clear();
					}

					if (this->Brake != nullptr)
					{
						delete this->Brake;
					}
					
					if (this->BrakeParameters != nullptr)
					{
						delete this->BrakeParameters;
					}

					if (this->AutomobileEngine != nullptr)
					{
						delete this->AutomobileEngine;
					}

					if (this->AEngineParameters != nullptr)
					{
						delete this->AEngineParameters;
					}

					if (this->PhysicsEngine != nullptr)
					{
						delete this->PhysicsEngine;
					}

					if (this->Environment != nullptr)
					{
						delete this->Environment;
					}
				}
			};

			void RunForTime(TTimePoint step, TTimePoint stop, vector<TestWorld*> worlds)
			{
				DebugClock* clk = static_cast<DebugClock*>(ClockFactory::get());

				while (clk->nowNanos() < stop)
				{
					clk->step();

					for (TestWorld* world : worlds)
					{
						world->Environment->update();
						world->Automobile->update();
						world->PhysicsEngine->update();
					}
				}
			}

			void RunForTime(TTimePoint step, TTimePoint stop, TestWorld* world)
			{
				vector<TestWorld*> worlds;
				worlds.push_back(world);
				RunForTime(step, stop, worlds);
			}

			/*These overloads are necesssary because numbers expressed in scientific notation are stored as doubles by default.*/
			/*The suffix 'LL' doesn't seem to compile, so define overloads to static_cast to correct type and avoid compile-time warnings.*/
			void RunForTime(double step, double stop, TestWorld* world)
			{
				RunForTime(static_cast<TTimePoint>(step), static_cast<TTimePoint>(stop), world);
			}

			void RunForTime(double step, double stop, vector<TestWorld*> worlds)
			{
				RunForTime(static_cast<TTimePoint>(step), static_cast<TTimePoint>(stop), worlds);
			}

			void ResetClock()
			{
				std::shared_ptr<DebugClock> clk = std::make_shared<DebugClock>(static_cast<TTimePoint>(0), static_cast<TTimePoint>(20E6));
				ClockFactory::get(clk);
			}
		};

}}

#endif