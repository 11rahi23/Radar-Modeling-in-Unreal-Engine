#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/LineBatchComponent.h"
#include "CustomTypes.h"
#include "RadarActor.generated.h"

#define UNVISITED 0
#define CORE 1
#define BOUNDARY 2
#define NOISE -2

UCLASS()
class RADAR_API ARadarActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARadarActor();
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = true))
		UStaticMeshComponent* BaseMesh; // Base Mesh, represents the sensor in 3D space

	// Radar manufacturer properties//
	UPROPERTY(EditDefaultsOnly, Category = "Radar Manufacturer Specs", meta = (AllowPrivateAccess = true))
		float frequency = 76.5f; // Frequency of the FMCW radar

	UPROPERTY(EditDefaultsOnly, Category = "Radar Manufacturer Specs", meta = (AllowPrivateAccess = true))
		float transmittedPower = 0.01f; // The maximum distance at which the sensor can see

	UPROPERTY(EditDefaultsOnly, Category = "Radar Manufacturer Specs", meta = (AllowPrivateAccess = true))
		float SNR = 25.0f;

	UPROPERTY(EditDefaultsOnly, Category = "Radar Manufacturer Specs", meta = (AllowPrivateAccess = true))
		float antennaGain = 100.0f;
	//Radar manufacturer property ends//

	UPROPERTY(EditDefaultsOnly, Category = "Range Variables", meta = (AllowPrivateAccess = true))
		float maxDistance = 6000.0f; // The maximum distance at which the sensor can see

	UPROPERTY(EditDefaultsOnly, Category = "Range Variables", meta = (AllowPrivateAccess = true))
		float minDistance = 100.0f; // The minimum distance at which the sensor can see

	UPROPERTY(EditDefaultsOnly, Category = "Range Variables", meta = (AllowPrivateAccess = true))
		FRange horizontalRange; // The struct containing the Yaw Range

	UPROPERTY(EditDefaultsOnly, Category = "Range Variables", meta = (AllowPrivateAccess = true))
		FRange verticalRange; // The struct containing the Pitch Range

	UPROPERTY(EditAnywhere, Category = "General")
		bool ShouldTick = true;

	UPROPERTY(EditAnywhere, Category = "General")
		bool drawPointsInWorld = true;

	UPROPERTY(EditInstanceOnly, Category = "General")
		FName ActionMappingName = "";

	UPROPERTY(EditDefaultsOnly, Category = "General")
		FColor DebugColor;

	UPROPERTY(EditDefaultsOnly, Category = "DBSCAN")
		FColor CorepointColor;

	UPROPERTY(EditDefaultsOnly, Category = "DBSCAN")
		FColor BorderpointColor;

	UPROPERTY(EditDefaultsOnly, Category = "DBSCAN")
		FColor CentroidpointColor;

	UPROPERTY(EditDefaultsOnly, Category = "DBSCAN")
		FColor NoisepointColor;

	UPROPERTY(EditDefaultsOnly, Category = "Properties")
		float PointLifeTime = 0.16f; // lifetime in seconds

	UPROPERTY(EditDefaultsOnly, Category = "Properties")
		float PointSize = 5.0f;

	UPROPERTY(EditDefaultsOnly, Category = "Properties")
		int maxPointsPerTick = 500;

	UPROPERTY(EditDefaultsOnly, Category = "Properties")
		int maxObjectsDetected = 20;

	UPROPERTY(EditDefaultsOnly, Category = "DBSCAN")
		int minimumSamples = 20;

	UPROPERTY(EditDefaultsOnly, Category = "DBSCAN")
		float epsilon = 6.0f;

	UPROPERTY(EditInstanceOnly, Category = "DBSCAN")
		bool exportPoints = false;

	float currH = 0.0f;
	float currV = 0.0f;
	int numHPoints = 0;
	int numVPoints = 0;
	int totalPoints = 0;
	ULineBatchComponent* LBatcher;
	bool isScanComplete = false;
	TQueue<FScan> scanQueue;
	FScan currScan;

	float cubeX;
	float cubeY;
	float cubeZ;
	float cubeSideLength;
	int numX;
	int numY;
	int numZ;
	int totalGrids;
	TArray<FGrid> gridArray;
	TArray<FRadarObject> prevObjects;
	TArray<FRadarObject> currObjects;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	void BindToInput();
	void Toggle();
	void ShowDBSCANResult();
	void CastVector(float h, float v);
	void RunDBSCAN();
	void CleanGrid();
	void BuildObjects(int objId, int cellIdx, int depth);
	void ExportPointCloudToTextFile(FString FileName, TArray<FRadarObject> Objects);
};
