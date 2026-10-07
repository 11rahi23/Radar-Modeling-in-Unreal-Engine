
#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "CustomTypes.generated.h"

#define UNVISITED 0
#define CORE 1
#define BOUNDARY 2
#define NOISE -2


#define DEBUGSTRING(fstring,x,time) if(GEngine) GEngine->AddOnScreenDebugMessage(-1, time, FColor::Yellow,FString::Printf(TEXT(fstring), x));

USTRUCT(Blueprintable)
struct UTILITIES_API FRange
{
	GENERATED_USTRUCT_BODY()
		UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Range Values", meta = (AllowPrivateAccess = true))
		float initialAngle = 0.0f;
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Range Values", meta = (AllowPrivateAccess = true))
		float finalAngle = 30.0f;
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Range Values", meta = (AllowPrivateAccess = true))
		float stepSize = 5.0f;
};

USTRUCT()
struct UTILITIES_API FPoint {
	GENERATED_BODY()

	UPROPERTY()
		FVector3d position;
	UPROPERTY()
		float dotToActorForward;
	UPROPERTY()
		FVector3d normal;
	UPROPERTY()
		float reflectivity;

	UPROPERTY()
		bool isEndPoint;

	FVector3d upLeftPoint = FVector3d::ZeroVector;
	FVector3d downLeftPoint = FVector3d::ZeroVector;
	FVector3d upRightPoint = FVector3d::ZeroVector;
	FVector3d downRightPoint = FVector3d::ZeroVector;

	FPoint(FVector3d _position, FVector3d _normal, float _dot, float _r, bool _isEndPoint) : position(_position), dotToActorForward(_dot), normal(_normal), reflectivity(_r), isEndPoint(_isEndPoint) {}
	FPoint(FVector3d _position, bool _isEndPoint) : position(_position), isEndPoint(_isEndPoint) {}
	FPoint() {
		position = FVector3d::ZeroVector;
		dotToActorForward = 0.0f;
		normal = FVector3d::ZeroVector;
		reflectivity = 1.0f;
		isEndPoint = false;
	}

	double distance(const FPoint& other) const {
		float dx = position.X - other.position.X;
		float dy = position.Y - other.position.Y;
		float dz = position.Z - other.position.Z;
		return sqrt(dx * dx + dy * dy + dz * dz);
	}
};

USTRUCT()
struct UTILITIES_API FRadarObject {
	GENERATED_BODY()
	
	UPROPERTY()
		int objID = 0;
	UPROPERTY()
		FString objStatus = "p";
	UPROPERTY()
		FString mv = "v";
	UPROPERTY()
		float range = 0.0f;
	UPROPERTY()
		float lateral = 0.0f;
	UPROPERTY()
		float rangeRate = 0.0f;
	UPROPERTY()
		float lateralRate = 0.0f;
	UPROPERTY()
		float acceleration = 0.0f;
	UPROPERTY()
		float object_width = 0.0f;
	UPROPERTY()
		float azimuthAngle = 0.0f;
	UPROPERTY()
		FPoint center = FPoint();
	UPROPERTY()
		TArray<FPoint> centroids = TArray<FPoint>();

	FRadarObject(int _objID) {
		objID = _objID;
	}

	FRadarObject() {}

	void calculateObjectCenter() {
		center.position = FVector3d::ZeroVector;
		for (int i = 0; i < centroids.Num(); i++) {
			center.position += centroids[i].position;
		}
		if (centroids.Num() > 0) center.position /= centroids.Num();
	}

	void FindRangeRateAndLateralVelocity(const FRadarObject& other, float scanTime, FVector actorForward, FVector actorRight) {
		float dx = center.position.X - other.center.position.X;
		float dy = center.position.Y - other.center.position.Y;
		float dz = center.position.Z - other.center.position.Z;
		FVector displacement = FVector(dx, dy, dz);
		rangeRate = dx / scanTime;
		lateralRate = dy / scanTime;
		range = center.position.Length();
		lateral = center.position.Y;
	}

	void FindAcceleration(const FRadarObject& other, float scanTime) {
		float dvx = rangeRate - other.rangeRate;
		acceleration = dvx / scanTime;
	}

	void FindStatus() {
		if (rangeRate == 0) {
			objStatus = "s";
		}
		else if (rangeRate > 0) {
			objStatus = "m";
			if (rangeRate > 6) {
				mv = "fv";
			}
			else {
				mv = "v";
			}
		}
		else if (rangeRate < 0) {
			objStatus = "o";
			if (rangeRate < -6) {
				mv = "fv";
			}
			else {
				mv = "v";
			}
		}
	}

	void FindAzimuth(FVector actorRight) {
		azimuthAngle = FMath::Asin(lateral / range) * 180 / PI;
	}

	void clean() {
		centroids.Empty();
		objStatus = "p";
		mv = "v";
		range = 0.0f;
		lateral = 0.0f;
		rangeRate = 0.0f;
		lateralRate = 0.0f;
		acceleration = 0.0f;
		object_width = 0.0f;
		azimuthAngle = 0.0f;
	}
};

// For Ultrasonic Sensor ROS topic

USTRUCT()
struct UTILITIES_API FUltrasonicObject {
	GENERATED_BODY()
	UPROPERTY()
		int objID = 0;
	UPROPERTY()
		int radiation_type = 0;
	UPROPERTY()
		float field_of_view = 0.0f;
	UPROPERTY()
		float min_range = 0.0f;
	UPROPERTY()
		float max_range = 0.0f;
	UPROPERTY()
		float range = 0.0f;
	UPROPERTY()
		FPoint center = FPoint();
	UPROPERTY()
		TArray<FPoint> centroids = TArray<FPoint>();

	FUltrasonicObject(int _objID) {
		objID = _objID;
	}

	FUltrasonicObject() {

	}

	void calculateObjectCenter() {
		center.position = FVector3d::ZeroVector;
		for (int i = 0; i < centroids.Num(); i++) {
			center.position += centroids[i].position;
		}
		if (centroids.Num() > 0) center.position /= centroids.Num();
	}


	void clean() {
		centroids.Empty();
		radiation_type = 0;
		field_of_view = 0.0f;
		min_range = 0.0f;
		max_range = 0.0f;
		range = 0.0f;
	}
};

USTRUCT()
struct UTILITIES_API FGrid {
	GENERATED_BODY()

	UPROPERTY()
		int X;
	UPROPERTY()
		int Y;
	UPROPERTY()
		int Z;
	UPROPERTY()
		int status;
	UPROPERTY()
		int objID;			// start using object ids only when creating the object recursively. 
	UPROPERTY()
		bool visited;	// use this flag only when creating the object recursively. 
	UPROPERTY()
		TArray<FPoint> children;
	UPROPERTY()
		FPoint childrenCentroid;
	UPROPERTY()
		float R;		// range of each grid centroid in meters
	UPROPERTY()
		float sigma;	//geometric radar cross-section of each centroid.
	UPROPERTY()
		float PrToN;		//Received Power
	UPROPERTY()
		float reflectivity;
	//UPROPERTY()
		//float directivity;

	UPROPERTY()
		float width;	// width of the grid
	UPROPERTY()
		float height;	// height of the grid

	UPROPERTY()
		float N;

	FGrid() {
		status = NOISE;
		objID = 0;
		visited = false;
		children = TArray<FPoint>();
		childrenCentroid = FPoint();
		X = 0, Y = 0, Z = 0;
		R = 0.0f;
		sigma = 0.0f;
		PrToN = 0.0f;
		width = 0.0f;
		height = 0.0f;
		reflectivity = 1.0f;
		N = 4e-15;
	}

	FGrid(int x, int y, int z) {
		X = x;
		Y = y;
		Z = z;
		status = NOISE;
		objID = 0;
		visited = false;
		children = TArray<FPoint>();
		childrenCentroid = FPoint();
		R = 0.0f;
		sigma = 0.0f;
		PrToN = 0.0f;
		width = 0.0f;
		height = 0.0f;
		reflectivity = 1.0f;
		N = 4e-15;
	}

	void calculateChildrenCentroid() {
		childrenCentroid.position = FVector3d::ZeroVector;
		for (int i = 0; i < children.Num(); i++) {
			childrenCentroid.position += children[i].position;
			childrenCentroid.normal += children[i].normal;
		}
		if (children.Num() > 0) childrenCentroid.position /= children.Num();
		if (children.Num() > 0) childrenCentroid.normal /= children.Num();
	}

	void calculateGridRange() {
		R = childrenCentroid.position.Size()/100;
	}

	void calculateWidthAndHeight() {
		float wMin = children[0].position.Y;
		float wMax = children[0].position.Y;
		float hMin = children[0].position.Z;
		float hMax = children[0].position.Z;
		for (FPoint const child : children) {
			if (child.position.Y < wMin) wMin = child.position.Y;
			if (child.position.Y > wMax) wMax = child.position.Y;
			if (child.position.Z < hMin) hMin = child.position.Z;
			if (child.position.Z > hMax) hMax = child.position.Z;
		}
		width = abs(wMax - wMin)/100;
		height = abs(hMax - hMin)/100;
	}

	void CalculateGridReflectivity() {
		for (FPoint const child : children) {
			reflectivity += child.reflectivity;
		}
		if (children.Num() > 0) reflectivity /= children.Num();
	}

	void calculateRadarCrossSection(float cubY, float cubZ, float eps, float freq) {
		float sigma_p = 0;
		//float areaRatio = 1 - (sqrt(((Y+0.5)*eps/sqrt(3)-cubY/2 - childrenCentroid.position.Y) * ((Y + 0.5) * eps / sqrt(3) - cubY / 2 - childrenCentroid.position.Y) + ((Z + 0.5) * eps / sqrt(3) - cubZ / 2 - childrenCentroid.position.Z) * ((Z + 0.5) * eps / sqrt(3) - cubZ / 2 - childrenCentroid.position.Z)))/sqrt((eps/sqrt(3))*(eps/sqrt(3)));
		float theta = acos(FVector::DotProduct(-childrenCentroid.position, childrenCentroid.normal) / (childrenCentroid.position.Size() * childrenCentroid.normal.Size()));
		//sigma = (4 * PI * FMath::Pow(((eps/100) / FMath::Sqrt(3)), 4.0f)) / (FMath::Pow((3e8 / (freq * 1e9)),2))*cos(theta)*areaRatio;
		if (width < height && width < 0.7 * eps / sqrt(3)) {
			sigma_p = 2 * PI * (width / 2) * height * height / (3e8 / (freq*1e9));		//considering the object as a vertical cylinder
			sigma = sigma_p / 2 * exp(-(0.1) * abs(theta)) ;							//physical cross-section*directivity
		}
		else if (width > height && height < 0.7 * eps / sqrt(3)) {
			sigma_p = 2 * PI * (height / 2) * width * width / (3e8 / (freq*1e9));		//considering the object as a horizontal cylinder
			sigma = sigma_p / 2 * exp(-(0.1) * abs(theta));								//physical cross-section*directivity
		}
		else {
			sigma_p = (4 * PI * FMath::Pow(((eps / 100) / FMath::Sqrt(3.0)), 4.0)) / (FMath::Pow((3e8 / (freq * 1e9)), 2)) * cos(theta);	//considering the object as a flat plate
			sigma = sigma_p / 2 * exp(-(0.0307 * log(reflectivity) + 0.3911) * abs(theta));		//physical cross-section*directivity
		}
		sigma *= reflectivity;															//physical cross-section*directivity*reflectivity
	}

	void calculateSignalToNoiseRatio(float pow, float freq, float gain) {
		//float theta = acos(FVector::DotProduct(-childrenCentroid.position, childrenCentroid.normal) / (childrenCentroid.position.Size() * childrenCentroid.normal.Size()));
		float Pr = (pow*gain * gain * FMath::Pow((3e8 / (freq * 1e9)), 2) * abs(sigma)) / (FMath::Pow(4 * PI, 3)* FMath::Pow(R, 4));
		PrToN = Pr / N;
	}

	void clean() {
		children.Empty();
		status = NOISE;
		objID = 0;
		visited = false;
		R = 0.0f;
		sigma = 0.0f;
		PrToN = 0.0f;
		width = 0.0f;
		height = 0.0f;
	}
};

USTRUCT()
struct UTILITIES_API FScan {
	GENERATED_BODY()

		UPROPERTY()
		TArray<FPoint> scannedPoints;
	UPROPERTY()
		float scanTime;
	UPROPERTY()
		unsigned int scanId;

	FScan() {
		scannedPoints = TArray<FPoint>();
		scanTime = 0.0f;
		scanId = 0;
	}
};



/**
 * 
 */
class UTILITIES_API CustomTypes
{
public:
	CustomTypes();
	~CustomTypes();
};
