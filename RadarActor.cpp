#include "RadarActor.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "SensorPhysicsMaterial.h"
#include <fstream>

// Sets default values
ARadarActor::ARadarActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	BaseMesh = CreateDefaultSubobject<UStaticMeshComponent>("Sensor Mesh"); // attach base mesh as root component, with the name "Sensor Mesh"
	RootComponent = BaseMesh;
	LBatcher = CreateDefaultSubobject<ULineBatchComponent>("Line Batcher");
}

// Called when the game starts or when spawned
void ARadarActor::BeginPlay()
{
	Super::BeginPlay();
	BindToInput();
	currH = horizontalRange.initialAngle;
	currV = verticalRange.initialAngle;

	numHPoints = (int)((horizontalRange.finalAngle - horizontalRange.initialAngle) / horizontalRange.stepSize);
	numVPoints = (int)((verticalRange.finalAngle - verticalRange.initialAngle) / verticalRange.stepSize);
	totalPoints = numHPoints * numVPoints;

	// Initialize the grid array
	cubeX = maxDistance;
	cubeY = 2 * (maxDistance * FMath::Tan(horizontalRange.finalAngle * PI / 180));
	cubeZ = 2 * (maxDistance * FMath::Tan(verticalRange.finalAngle * PI / 180));
	numX = FMath::CeilToInt((cubeX * FMath::Sqrt(3.0)) / epsilon);
	numY = FMath::CeilToInt((cubeY * FMath::Sqrt(3.0)) / epsilon);
	numZ = FMath::CeilToInt((cubeZ * FMath::Sqrt(3.0)) / epsilon);
	totalGrids = numX * numY * numZ;
	cubeSideLength = epsilon / FMath::Sqrt(3.);
	for (int i = 0; i < numX; i++) {
		for (int j = 0; j < numY; j++) {
			for (int k = 0; k < numZ; k++) {
				gridArray.Add(FGrid(i, j, k));
			}
		}
	}
	for (int i = 0; i < maxObjectsDetected; i++) {
		prevObjects.Add(FRadarObject(i));
		currObjects.Add(FRadarObject(i));
	}
}

// Called every frame
void ARadarActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	for (int i = 0; i < maxPointsPerTick; i++) {
		CastVector(currH, currV); // Cast Vector using current angle positions
		currV += verticalRange.stepSize; // increment vertical angle
		if (currV > verticalRange.finalAngle) { // check if vertical angle is greater than maximum range
			currV = verticalRange.initialAngle; // reset vertical angle
			currH += horizontalRange.stepSize; // increment horizontal angle
			if (currH > horizontalRange.finalAngle) { // check if horizontal angle is greater than maximum range
				currH = horizontalRange.initialAngle;
				if (GEngine && currScan.scanTime != 0.0f) GEngine->AddOnScreenDebugMessage(-1, GetWorld()->DeltaTimeSeconds * 2, FColor::Yellow, FString::Printf(TEXT("Radar frequency: %f"), 1 / currScan.scanTime));
				//isScanComplete = true;
				//scanQueue.Enqueue(currScan);
				RunDBSCAN();
				prevObjects = currObjects;
				ShowDBSCANResult();
				if (exportPoints) ExportPointCloudToTextFile("Output.csv",currObjects);
				currScan.scannedPoints.Empty();
				currScan.scanId += 1;
				currScan.scanTime = 0.0f;
				CleanGrid();
			}
		}
	}
	currScan.scanTime += GetWorld()->DeltaTimeSeconds;
}

void ARadarActor::BindToInput()
{
	InputComponent = NewObject<UInputComponent>(this);
	InputComponent->RegisterComponent();
	if (InputComponent) {
		InputComponent->BindAction(ActionMappingName, IE_Pressed, this, &ARadarActor::Toggle);
		EnableInput(GetWorld()->GetFirstPlayerController());
	}
}

void ARadarActor::Toggle()
{
	if (ShouldTick) {
		ShouldTick = false;
		SetActorTickEnabled(false);
	}
	else {
		ShouldTick = true;
		SetActorTickEnabled(true);
	}
	currH = horizontalRange.initialAngle;
	currV = verticalRange.initialAngle;
}

void ARadarActor::ShowDBSCANResult()
{
	for (FGrid& grid : gridArray) {
		if (grid.status == CORE) {
			FVector pointInWorldFrame = GetTransform().TransformPosition(grid.childrenCentroid.position);
			LBatcher->DrawPoint(pointInWorldFrame, CorepointColor, PointSize, 0, PointLifeTime);
		}
	}
	for (FRadarObject& object : currObjects) {
		if (object.centroids.Num() > 0) {
			FVector pointInWorldFrame = GetTransform().TransformPosition(object.center.position);
			LBatcher->DrawPoint(pointInWorldFrame, CentroidpointColor, PointSize * 2, 0, PointLifeTime);
		}
	}
}

void ARadarActor::CastVector(float h, float v)
{
	FHitResult outHit; // hit result struct that will be passed values
	FVector actorLocation = GetActorLocation();
	FVector maxDistVector = GetActorForwardVector() * maxDistance;
	FVector minDistVector = GetActorForwardVector() * minDistance;
	maxDistVector = maxDistVector.RotateAngleAxis(v, GetActorRightVector());
	maxDistVector = maxDistVector.RotateAngleAxis(h, GetActorUpVector());
	minDistVector = minDistVector.RotateAngleAxis(v, GetActorRightVector());
	minDistVector = minDistVector.RotateAngleAxis(h, GetActorUpVector());
	FVector start = actorLocation + minDistVector;
	FVector end = actorLocation + maxDistVector;

	FCollisionQueryParams collisionParams; // custom collision parameters
	collisionParams.AddIgnoredActor(this->GetOwner()); //tell the ray cast to ignore collision with this actor
	collisionParams.AddIgnoredComponent(BaseMesh);
	collisionParams.bReturnPhysicalMaterial = true;

	bool isHit = GetWorld()->LineTraceSingleByChannel(outHit, start, end, ECC_WorldDynamic, collisionParams); // returns true if collision is detected on the line trace
	//DrawDebugLine(GetWorld(), start, outHit.bBlockingHit ? outHit.ImpactPoint : start, outHit.bBlockingHit ? FColor::Blue : FColor::Red, false, 5.0f, 0, 10.0f);
	//DrawDebugPoint(GetWorld(), outHit.bBlockingHit ? outHit.ImpactPoint : end, 5, FColor::Yellow);
	LBatcher->DrawPoint(outHit.ImpactPoint, DebugColor, PointSize / 2, 0, PointLifeTime);

	if (isHit) { // if an actor is hit
		FVector3d hitLocation = outHit.ImpactPoint - actorLocation; // gets the coordinates of collision
		FVector3d objectNormal = outHit.Normal.GetSafeNormal();

		FVector3d DirectionVector = (-1 * hitLocation).GetSafeNormal(); //gets the direction vector of the ray cast
		float dot = objectNormal.Dot(this->GetActorForwardVector());
		float incidentAngle = acos(FVector::DotProduct(DirectionVector, objectNormal)) * 180 / PI; //dot product between hitLocation normal and direction vector of ray cast
		float reflectivity = 1.0f;

		TWeakObjectPtr<UPhysicalMaterial> weakPhysxPointer = outHit.PhysMaterial;
		if (weakPhysxPointer.IsValid()) {
			UPhysicalMaterial* hitPhysMaterial = weakPhysxPointer.Get();
			USensorPhysicsMaterial* radarPhysxMat = Cast<USensorPhysicsMaterial>(hitPhysMaterial);
			if (radarPhysxMat) {
				reflectivity = radarPhysxMat->RadarReflectivity; // if the pointer is not null then the cast worked. 
			}
		}

		if (abs(incidentAngle) < 90) {
			FTransform actorTransform = GetActorTransform();
			FQuat actorRotation = actorTransform.GetRotation();
			FQuat inverseRotation = actorRotation.Inverse();
			FVector localHitLocation = inverseRotation.RotateVector(hitLocation);
			currScan.scannedPoints.Add(FPoint(localHitLocation, objectNormal, dot, reflectivity, false));
		}
	}
}

void ARadarActor::RunDBSCAN()
{
	for (FPoint& point : currScan.scannedPoints) {
		int xidx = FMath::FloorToInt(point.position.X * FMath::Sqrt(3.) / epsilon);
		int yidx = FMath::FloorToInt((point.position.Y + (cubeY / 2)) * FMath::Sqrt(3.) / epsilon);
		int zidx = FMath::FloorToInt((point.position.Z + (cubeZ / 2)) * FMath::Sqrt(3.) / epsilon);
		int gridIdx = zidx + numZ * (yidx + numY * xidx);
		gridArray[gridIdx].children.Add(point);
	}
	for (FGrid& grid : gridArray) {
		if (grid.children.Num() > 0) {
			grid.calculateChildrenCentroid();
			grid.calculateWidthAndHeight();
			grid.calculateGridRange();
			grid.calculateRadarCrossSection(cubeY, cubeZ, epsilon, frequency);
			grid.calculateSignalToNoiseRatio(transmittedPower, frequency, antennaGain);
			if (grid.children.Num() >= minimumSamples && grid.PrToN>SNR) { //added SNR portion here. Also change ends here */
			//if (grid.children.Num() >= minimumSamples) {
				grid.status = CORE;
			}
		}
	};

	int objID = 0;
	//Recursively build the objects
	for (int i = 0; i < gridArray.Num(); i++) {
		if (gridArray[i].status != NOISE && !gridArray[i].visited) {
			if (objID < maxObjectsDetected) {
				BuildObjects(objID, i, 8);
				currObjects[objID].calculateObjectCenter();
				objID++;
			}
		}
	}

	// Calculate object status, center, velocity and acceleration
	for (FRadarObject& prevObj : prevObjects) {
		for (FRadarObject& currObj : currObjects) {
			double d = currObj.center.distance(prevObj.center);
			if (d < 150) {
				//same object from the previous scan and it has moved distance d
				currObj.FindRangeRateAndLateralVelocity(prevObj, currScan.scanTime, FVector3d(1, 0, 0), FVector3d(0, 1, 0));
				currObj.FindAcceleration(prevObj, currScan.scanTime);
				currObj.FindStatus();
				currObj.FindAzimuth(FVector3d(0, 1, 0));				
			}
		}
	}
}

void ARadarActor::CleanGrid()
{
	for (FGrid& grid : gridArray) {
		grid.clean();
	}
	for (FRadarObject& object : currObjects) {
		object.clean();
	}
}

void ARadarActor::BuildObjects(int objId, int cellIdx, int depth)
{
	if (depth == 0) {
		return; // Stop recursion if max depth is reached
	}
	
	gridArray[cellIdx].objID = objId;
	currObjects[objId].centroids.Add(gridArray[cellIdx].childrenCentroid);
	for (int32 k = -1; k <= 1; k++) {
		for (int32 j = -1; j <= 1; j++) {
			for (int32 i = -1; i <= 1; i++) {
				if (i == 0 && j == 0 && k == 0) {
					continue;
				}
				int32 NeighborX = gridArray[cellIdx].X + i;
				int32 NeighborY = gridArray[cellIdx].Y + j;
				int32 NeighborZ = gridArray[cellIdx].Z + k;

				//Check if the neighbor is within the bounds of the grid
				if (NeighborX < 0 || NeighborX >= numX ||
					NeighborY < 0 || NeighborY >= numY ||
					NeighborZ < 0 || NeighborZ >= numZ) {
					continue;
				}
				if (gridArray[NeighborZ + numZ * (NeighborY + numY * NeighborX)].status != NOISE && !gridArray[NeighborZ + numZ * (NeighborY + numY * NeighborX)].visited) {
					BuildObjects(objId, NeighborZ + numZ * (NeighborY + numY * NeighborX), depth - 1);
				}
			}
		}
	}
	gridArray[cellIdx].visited = true;
}

void ARadarActor::ExportPointCloudToTextFile(FString FileName, TArray<FRadarObject> Objects)
{
	FString SaveDirectory = FPaths::ProjectSavedDir();
	FString AbsoluteFilePath = SaveDirectory + "/" + FileName;

	// Open the file for writing
	std::fstream OutFile(TCHAR_TO_UTF8(*AbsoluteFilePath), std::ios::app);

	// Write the header
	//OutFile << "Actor location: " << this->GetActorLocation().X << " " << this->GetActorLocation().Y << " " << this->GetActorLocation().Z << std::endl;
	//OutFile << "x,y,z" << std::endl;

	// Write the point cloud data
	for (FRadarObject& Object : Objects)
	{
		OutFile << Object.objID << "," << Object.range << "," << Object.azimuthAngle << std::endl;
	}

	// Close the file
	OutFile.close();
}
