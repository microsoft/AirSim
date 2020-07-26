//*****************************************************************************
//* File: Operations.cs
//* Project: Firefly (Microsoft Hackaton 2020)
//* Description: Demo of AOLUSS REST Client
//* Author: Mark West (mark.west@microsoft.com)
//*****************************************************************************

namespace AolussClientConsole
{
    using System;
    using System.Collections.Generic;
    using AolussClientLib.Client;
    using AolussClientLib.Api;
    using AolussClientLib.Model;

    /// <summary>
    /// Demo of AolussClient Operations
    /// </summary>
    /// <remarks>
    /// Demo of AolussClient Operations
    /// </remarks>
    public class Operations
    {
        private OperationsApi instance;
        private bool isInitialized = false;

        const string UNINIT_MSG = "Call Initialize() before invoking an operation";

        #region init

        /// <summary>
        /// Constructor
        /// </summary>
        public Operations()
        {
        }

        /// <summary>
        /// Initialize
        /// </summary>
        public void Initialize()
        {
            try
            {
                instance = new OperationsApi();
                isInitialized = true;
            }
            catch (Exception ex)
            {
                throw new MessageException("PutOperatorOperation Failed", ex);
            }
        }

        #endregion init

        #region operations

        /// <summary>
        /// Delete Operator Operation
        /// </summary>
        public ResponseOk DeleteOperatorOperation(Guid gufi)
        {
            if (!isInitialized) 
                throw new InvalidOperationException(UNINIT_MSG);

            try
            {
                return instance.DeleteOperatorOperation(gufi);
            }
            catch (Exception ex)
            {
                throw new MessageException("PutOperatorOperation Failed", ex);
            }
        }

        /// <summary>
        /// Get Operator Operation
        /// </summary>
        public void GetOperatorOperation(Guid gufi)
        {
            if (!isInitialized)
                throw new InvalidOperationException(UNINIT_MSG);

            try
            {
                instance.GetOperatorOperation(gufi);
            }
            catch (Exception ex)
            {
                throw new MessageException("PutOperatorOperation Failed", ex);
            }
        }

        /// <summary>
        /// Post an Operator Message
        /// </summary>
        public void PostOperatorMessage(ModelOperatorMessage payload)
        {
            if (!isInitialized)
                throw new InvalidOperationException(UNINIT_MSG);

            try
            {
                instance.PostOperatorMessage(payload);
            }
            catch (Exception ex)
            {
                throw new MessageException("PutOperatorOperation Failed", ex);
            }
        }

        /// <summary>
        /// Post an operator position
        /// </summary>
        public void PostOperatorPosition(ModelOperatorPosition payload)
        {
            if (!isInitialized)
                throw new InvalidOperationException(UNINIT_MSG);

            try
            {
                instance.PostOperatorPosition(payload);
            }
            catch (Exception ex)
            {
                throw new MessageException("PutOperatorOperation Failed", ex);
            }
        }

        /// <summary>
        /// PutOperatorOperation
        /// </summary>
        public void PutOperatorOperation(ModelOperatorOperation payload)
        {
            if (!isInitialized)
                throw new InvalidOperationException(UNINIT_MSG);

            try
            {
                instance.PutOperatorOperation(payload);
            }
            catch(Exception ex)
            {
                throw new MessageException("PutOperatorOperation Failed", ex);
            }
        }

        #endregion operations

        #region DemoMessages

        /// <summary>
        /// Post an Operator Message
        /// </summary>
        public ModelOperatorMessage GenOperatorMessage()
        {
            return new ModelOperatorMessage
            {
                FreeText = "",
                Gufi = new Guid(),
                MessageId = new Guid(),
                MessageType = ModelOperatorMessage.MessageTypeEnum.OPERATIONROGUE,
                PrevMessageId = new Guid(),
                Severity = ModelOperatorMessage.SeverityEnum.EMERGENCY,
                TimeSent = DateTime.Now
            };
        }

        /// <summary>
        /// Post an operator position
        /// </summary>
        public ModelOperatorPosition GenOperatorPosition()
        {
            return new ModelOperatorPosition
            {
                AltitudeGps = new ModelAltitudeFixm
                {
                    AltitudeValue = 0,
                    Source = ModelAltitudeFixm.SourceEnum.ONBOARDSENSOR,
                    UnitsOfMeasure = ModelAltitudeFixm.UnitsOfMeasureEnum.FT,
                    VerticalReference = ModelAltitudeFixm.VerticalReferenceEnum.W84
                },
                AltitudeNumGpsSatellites = 14,
                Comments = "This is a comment",
                EnroutePositionsId = new Guid("3fa85f64-5717-4562-b3fc-2c963f66afa6"),
                Gufi = new Guid("3fa85f64-5717-4562-b3fc-2c963f66afa6"),
                HdopGps = 0,
                Location = new GeojsonPoint2D
                {
                    // schema says Location.Coordinates is a 2D vector. I assume this is Lat Lon, not sure
                    Coordinates = { (decimal?)47.4685631, (decimal?)-121.7674447 },
                    Type = GeojsonPoint2D.TypeEnum.Point
                },
                TimeMeasured = DateTime.Now,
                TrackBearing = 0,
                TrackBearingReference = ModelOperatorPosition.TrackBearingReferenceEnum.TRUENORTH,
                TrackBearingUom = ModelOperatorPosition.TrackBearingUomEnum.DEG,
                TrackGroundSpeed = 0,
                TrackGroundSpeedUnits = ModelOperatorPosition.TrackGroundSpeedUnitsEnum.KT,
                VdopGps = 0
            };
        }

        /// <summary>
        /// PutOperatorOperation
        /// </summary>
        public ModelOperatorOperation GenOperatorOperation()
        {
            return new ModelOperatorOperation
            {
                AircraftComments = "This is a UAV",
                AirspaceAuthorization = new Guid(),
                Contact = new ModelContact
                {
                    Comments = "Excelsior!",
                    EmailAddresses = new List<string> { "Flyer@Mcflyface.com" },
                    Name = "Flyer McFlyface",
                    PhoneNumbers = new List<string> { "123.456.7890" }
                },
                ContingencyPlans = new List<ModelContingencyPlans>(),
                ControllerLocation = new GeojsonPoint2D
                {
                    Coordinates = new Point2D { (decimal?)47.4685631, (decimal?)-121.7674447 },
                    Type = GeojsonPoint2D.TypeEnum.Point
                },
                FaaRule = ModelOperatorOperation.FaaRuleEnum.PART107,
                FlightComments = "We shall fly",
                FlightNumber = "123",
                GcsLocation = new GeojsonPoint2D
                {
                    Coordinates = new Point2D { (decimal?)47.4685631, (decimal?)-121.7674447 },
                    Type = GeojsonPoint2D.TypeEnum.Point
                },
                Gufi = new Guid(),
                Metadata = new ModelMetadata
                {
                    CallSign = "123abc",
                    DataCollection = false,
                    EventId = "1",
                    FreeText = "Free Text",
                    Location = "Test Field 13",
                    Scenario = "Demo Flight",
                    Setting = ModelMetadata.SettingEnum.FIELD,
                    Source = ModelMetadata.SourceEnum.SWITL,
                    TestCard = "Test Card 13",
                    TestType = ModelMetadata.TestTypeEnum.FLIGHT
                },
                OperationVolumes = new List<ModelOperationVolumes>
                {
                    new ModelOperationVolumes
                    {
                        ActualTimeEnd = DateTime.Now,
                        BeyondVisualLineOfSight = false,
                        EffectiveTimeBegin = DateTime.Now,
                        EffectiveTimeEnd = DateTime.Now,
                        MaxAltitude = new ModelAltitudeFixm
                        {
                            AltitudeValue = 400,
                            Source = ModelAltitudeFixm.SourceEnum.ONBOARDSENSOR,
                            UnitsOfMeasure = ModelAltitudeFixm.UnitsOfMeasureEnum.FT,
                            VerticalReference = ModelAltitudeFixm.VerticalReferenceEnum.W84
                        },
                        MinAltitude = new ModelAltitudeFixm
                        {
                            AltitudeValue = 0,
                            Source = ModelAltitudeFixm.SourceEnum.ONBOARDSENSOR,
                            UnitsOfMeasure = ModelAltitudeFixm.UnitsOfMeasureEnum.FT,
                            VerticalReference = ModelAltitudeFixm.VerticalReferenceEnum.W84
                        },
                        NearStructure = false,
                        OperationGeography = new GeojsonPolygon2D(),
                        Ordinal = 1,
                        VolumeType = new ModelOperationVolumes.VolumeTypeEnum()
                    }
                },
                PriorityElements = new ModelPriorityElements
                {
                    PriorityLevel = ModelPriorityElements.PriorityLevelEnum.INFORMATIONAL,
                    PriorityStatus = ModelPriorityElements.PriorityStatusEnum.PUBLICSAFETY
                },
                UasRegistrations = new List<ModelUasRegistrations>
                {
                    new ModelUasRegistrations
                    {
                        RegistrationId = new Guid(),
                        RegistrationLocation = "Redmond, WA"
                    }
                },
                VolumesDescription = "We describe the volumes here"
            };
        }

        #endregion DemoMessages

        #region Demo

        public static void RunDemo()
        {
            //AolussClientLib.Client.Configuration.Default.AccessToken = "";
            //AolussClientLib.Client.Configuration.Default.BasePath = "";
            //AolussClientLib.Client.Configuration.Default.Password = "";
            //AolussClientLib.Client.Configuration.Default.Username = "";

            Operations ops = new Operations();
            ops.Initialize();

            ops.PutOperatorOperation(ops.GenOperatorOperation());
            ops.PostOperatorMessage(ops.GenOperatorMessage());
            ops.PostOperatorPosition(ops.GenOperatorPosition());
        }

        #endregion Demo
    }
}

