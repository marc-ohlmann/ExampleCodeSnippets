
#include <iostream>
#include <vector>
#include <string>

// struct implementation of a PID controller algorithm
// This struct acts both as a collection of data values, and defines functionality for using them.
// However, it is still expected that this struct's data (tunings especially) will be passed around
// between classes, and it's use will be governed by some managing class as well. For example, an AI 
// controller that uses multiple PID controllers to implement steering and locomotion.
//
// Once provided with PID gain values and a periodic duration, the managing class calls Tick() in some 
// repeating loop where the time between ticks is passed in, along with the error value (or raw target 
// and setpoint values for some improved functionality).
// 
// The calculated value from the PID controller can be retrieved by using GetLastCalculatedValue().
//
struct FPIDController
{
public:

	FPIDController()
	{
		_CalculationAverageBufferSize = 1;

		ClearState();

		ControlledValue_Max = 1.f;
		ControlledValue_Min = 0.f;

		PeriodicDuration = 0.2f;
		P_Gain = 1.f;
		I_Gain = 0.f;
		D_Gain = 0.f;
	}

	FPIDController(float InP_Gain, float InI_Gain, float InD_Gain, float MaxValue, float MinValue, float InPeriodicDuration)
		: FPIDController()
	{
		P_Gain = InP_Gain;
		I_Gain = InI_Gain;
		D_Gain = InD_Gain;
		ControlledValue_Max = MaxValue;
		ControlledValue_Min = MinValue;
		PeriodicDuration = InPeriodicDuration;
	}

	// reset properties related to the state of an active PID controller
	void ClearState()
	{
		_bIsEnabled = true;
		_TickBuffer = 0.f;
		_IntegralAccumulation = 0.f;
		_PreviousCalculation = 0.f;
		_PreviousInput = 0.f;
		_PreviousError = 0.f;

		ClearAveragingBuffer();
	}

	// proportional gain
		float P_Gain;

	// integral gain
		float I_Gain;

	// differential gain
		float D_Gain;

	// maximum value of the value that is being controlled
		float ControlledValue_Max;

	// minimum value of the value that is being controlled
		float ControlledValue_Min;

	// periodic duration (seconds)
	// determines operating frequency
	// use SetPeriodicDuration() to change this on the fly
		float PeriodicDuration;

	// calculate a new controlled value using the current value and the desired target setpoint to calculate error
	float CalculateNewValue(const float TargetSetpoint, const float CurrentValue, float DeltaTime);

	// calculate a new controlled value using the given error value
	// this version is susceptible to derivative kick, since it only provides a raw error value
	float CalculateNewValue(const float Error, float DeltaTime);

	// accumulates DeltaTime into the buffer and performs a calculation if it overflows
	// use GetLastCalculatedValue() to retrieve the calculated value
	// returns true if an overflow accured
	bool Tick(const float TargetSetpoint, const float CurrentValue, float DeltaTime);

	// ticks if the controller is active
	bool TickIfEnabled(const float TargetSetpoint, const float CurrentValue, float DeltaTime)
	{
		if (IsEnabled()) return Tick(TargetSetpoint, CurrentValue, DeltaTime);
		return false;
	}

	// accumulates DeltaTime into the buffer and performs a calculation if it overflows
	// use GetLastCalculatedValue() to retrieve the calculated value
	// returns true if an overflow accured
	// this version is susceptible to derivative kick, since it only provides a raw error value
	bool Tick(const float Error, float DeltaTime);

	// ticks if the controller is active
	// this version is susceptible to derivative kick, since it only provides a raw error value
	bool TickIfEnabled(const float Error, float DeltaTime)
	{
		if (IsEnabled()) return Tick(Error, DeltaTime);
		return false;
	}

	// use to retrieve the previously calculated value from CalculateNewValue()
	float GetLastCalculatedValue() const { return _PreviousCalculation; }

	// use to change periodic duration on the fly
	// modifies integral and differential gain values proportional to the duration change
	void SetPeriodicDuration(const float PeriodicDuration);

	// check if the PID controller is active
	bool IsEnabled() const { return _bIsEnabled; }

	// set the PID controller to be active / inactive
	// if enabling the controller, optionally clear the current integral accumulation
	void SetEnabled(bool IsEnabled, bool bClearIntegralAccumulation = false);

	// get error value used for the last calculated value
	float GetPreviousError() const { return _PreviousError; }

	// get previous input provided, only valid if providing a target setpoint AND current value 
	// to perform calculations. raw error input does not store previous input
	float GetPreviousInput() const { return _PreviousInput; }

	// get value of the current integral accumulation of error
	float GetIntegralAccumulation() const { return _IntegralAccumulation; }

	// use to retrieve the average of previously calculated values
	float GetAverageCalculatedValue() const;

	// change the size of the averaging buffer
	// will also clear the contents of the averaging buffer
	void SetAveragingBufferSize(const int AveragingBufferSize);

	// get the size of the averaging buffer
	int GetAveragingBufferSize() const { return _CalculationAverageBufferSize; }

private:

	// check if a floating point value is nearly zero, within a radius of tolerance
	static bool IsNearlyZero(float value)
	{
		static const float ZeroThresholdRadius = 0.00001f;
		return 	value == 0.f || 
				(value > -ZeroThresholdRadius && value < ZeroThresholdRadius); 
	}
	
	// accumulate time into a buffer, and return true if the buffer overflowed
	static bool AccumulateBuffer(float& Buffer, float DeltaTime, float BufferSize)
	{
		Buffer += DeltaTime;
		if (Buffer >= BufferSize)
		{
			Buffer -= BufferSize;
			return true;
		}


		return false;
	}

	// buffer used to average the output calculation value
		std::vector<float> _CalculationAveragingBuffer;

	// size of the buffer used for averaging
		int _CalculationAverageBufferSize;

	// initialize the averaging buffer
	void ClearAveragingBuffer();

	// cache the given calculated value as the previous calculation, and for averaging
	void CachePreviousCalculation(const float CalculatedValue);

	// initialize values that are valid while enabled
	void Initialize(bool bClearIntegralAccumulation = false);

	// flag indicates if this PID controller is active
		bool _bIsEnabled;

	// previous error
		float _PreviousError;

	// cache the given error value as the previous error
	void CachePreviousError(const float Error);

	// previously calculated value
		float _PreviousCalculation;

	// previous input value
		float _PreviousInput;

	// cache the given input value as the previous input
	void CachePreviousInput(const float InputValue);

	// current integral accumulation
		float _IntegralAccumulation;

	// accumulated tick time buffer for controlling PID frequency
		float _TickBuffer;

	// calculates and returns the 'P' value from the given error
	float ProportionalError(const float Error);

	// calculates and returns the 'I' value from the given error and time delta
	float IntegralError(const float Error, const float DeltaTime);

	// calculates and returns the 'D' value from the given error
	// this version is susceptible to derivative kick when the setpoint is adjusted
	float DifferentialError(const float Error, const float DeltaTime);

	// calculates and returns the 'D' value from the given error
	// this version prevents derivative kick when the setpoint is adjusted
	float DifferentialError_PreventDerivativeKick(const float CurrentInput, const float DeltaTime);

	// clamps result and caches the output calculation
	float ClampAndCacheOutput(float OutputValue);

};