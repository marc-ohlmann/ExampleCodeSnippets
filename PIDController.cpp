#include "PIDController.h"

float FPIDController::CalculateNewValue(const float TargetSetpoint, const float CurrentValue, float DeltaTime)
{
	if (FPIDController::IsNearlyZero(DeltaTime))
	{
		std::cout << "delta time nearly zero";
		return 0.f;
	}

	// calculate error
	const float Error = TargetSetpoint - CurrentValue;

	// calculate output
	float Output = 0.f;

	// proportional
	Output += ProportionalError(Error);

	// integral
	Output += IntegralError(Error, DeltaTime);

	// differential -- improved to prevent derivative kick
	Output += DifferentialError_PreventDerivativeKick(CurrentValue, DeltaTime);

	// cache error and current input value
	CachePreviousError(Error);
	CachePreviousInput(CurrentValue);


	return ClampAndCacheOutput(Output);
}


float FPIDController::CalculateNewValue(const float Error, float DeltaTime)
{
	if (FPIDController::IsNearlyZero(DeltaTime))
	{
		std::cout << "delta time nearly zero";
		return 0.f;
	}

	// calculate output
	float Output = 0.f;

	// proportional
	Output += ProportionalError(Error);

	// integral
	Output += IntegralError(Error, DeltaTime);

	// differential
	Output += DifferentialError(Error, DeltaTime);

	// save error to previous error
	CachePreviousError(Error);


	return ClampAndCacheOutput(Output);
}


float FPIDController::ProportionalError(const float Error)
{
	if (FPIDController::IsNearlyZero(P_Gain))
	{
		return 0.f;
	}


	return P_Gain * Error;
}


float FPIDController::IntegralError(const float Error, const float DeltaTime)
{
	if (FPIDController::IsNearlyZero(I_Gain) == true)
	{
		return _IntegralAccumulation;
	}

	// improvement -- gain applied here to prevent wacky behavior when tuning on the fly
	_IntegralAccumulation += I_Gain * Error * DeltaTime;

	// improvement -- clamp to prevent integral windup
	if (_IntegralAccumulation > ControlledValue_Max)
	{
		_IntegralAccumulation = ControlledValue_Max;
	}
	else if (_IntegralAccumulation < ControlledValue_Min)
	{
		_IntegralAccumulation = ControlledValue_Min;
	}


	return _IntegralAccumulation;
}


float FPIDController::DifferentialError(const float Error, const float DeltaTime)
{
	if (DeltaTime < 0.f || 
		FPIDController::IsNearlyZero(DeltaTime) ||
		FPIDController::IsNearlyZero(D_Gain))
	{
		return 0.f;
	}


	return D_Gain * ((Error - _PreviousError) / DeltaTime);
}


float FPIDController::DifferentialError_PreventDerivativeKick(const float CurrentInput, const float DeltaTime)
{
	if (DeltaTime < 0.f ||
		FPIDController::IsNearlyZero(DeltaTime) ||
		FPIDController::IsNearlyZero(D_Gain))
	{
		return 0.f;
	}


	// improvement -- derivative of error is equal to negative derivative of input -- prevents derivative kick
	return -1.f * D_Gain * ((CurrentInput - _PreviousInput) / DeltaTime);
}


void FPIDController::CachePreviousError(const float Error)
{
	_PreviousError = Error;


	return;
}


void FPIDController::CachePreviousInput(const float InputValue)
{
	_PreviousInput = InputValue;


	return;
}


float FPIDController::ClampAndCacheOutput(float OutputValue)
{
	// clamp to max/min
	if (OutputValue > ControlledValue_Max)
	{
		OutputValue = ControlledValue_Max;

	}
	else if (OutputValue < ControlledValue_Min)
	{
		OutputValue = ControlledValue_Min;
	}

	// cache calculation
	CachePreviousCalculation(OutputValue);


	return OutputValue;
}


void FPIDController::CachePreviousCalculation(const float CalculatedValue)
{
	_PreviousCalculation = CalculatedValue;

	if (GetAveragingBufferSize() > 1)
	{
		_CalculationAveragingBuffer.push_back(CalculatedValue);
		_CalculationAveragingBuffer.erase(_CalculationAveragingBuffer.begin());
	}


	return;
}


void FPIDController::ClearAveragingBuffer()
{
	_CalculationAveragingBuffer.clear();
	_CalculationAveragingBuffer.reserve(_CalculationAverageBufferSize);
	if (_CalculationAverageBufferSize > 0)
	{
		for(int i = 0; i < _CalculationAverageBufferSize; i++)
		{
			_CalculationAveragingBuffer.push_back(0.f);
		}
	}


	return;
}


void FPIDController::SetAveragingBufferSize(const int AveragingBufferSize)
{
	_CalculationAverageBufferSize = AveragingBufferSize;

	ClearAveragingBuffer();


	return;
}


float FPIDController::GetAverageCalculatedValue() const
{
	const int AveragingBufferSize = GetAveragingBufferSize();
	if (AveragingBufferSize <= 1)
	{
		return _PreviousCalculation;
	}

	float Average = 0.f;
	for(int i = 0; i < AveragingBufferSize; i++)
	{
		if (i >= _CalculationAveragingBuffer.size())
		{
			break;
		}

		Average += _CalculationAveragingBuffer[i];
	}
	Average /= (float)AveragingBufferSize;


	return Average;
}


bool FPIDController::Tick(const float TargetSetpoint, const float CurrentValue, float DeltaTime)
{
	if (PeriodicDuration > 0.f)
	{
		if (DeltaTime > PeriodicDuration)
		{
			//L_VERBOSE(MEOHelpers, "<TargetSetpoint, CurrentValue> tick time has exceeded PID periodic duration -- may produce unstable results");

			// last tick took longer than periodic duration
			// accumulate the full tick duration and calculate
			FPIDController::AccumulateBuffer(_TickBuffer, DeltaTime, DeltaTime);
			CalculateNewValue(TargetSetpoint, CurrentValue, DeltaTime);
			return true;
		}
		else if (FPIDController::AccumulateBuffer(_TickBuffer, DeltaTime, PeriodicDuration) == true)
		{
			// accumulate the periodic duration and calculate
			CalculateNewValue(TargetSetpoint, CurrentValue, PeriodicDuration);
			return true;
		}


		// no calculations this frame
		return false;
	}

	// periodic duration is undefined
	// calculate on every frame
	CalculateNewValue(TargetSetpoint, CurrentValue, DeltaTime);


	return true;
}


bool FPIDController::Tick(const float Error, float DeltaTime)
{
	if (PeriodicDuration > 0.f)
	{
		if (DeltaTime > PeriodicDuration)
		{
			//L_VERBOSE(MEOHelpers, "tick time has exceeded PID periodic duration -- may produce unstable results");

			// last tick took longer than periodic duration
			// accumulate the full tick duration and calculate
			FPIDController::AccumulateBuffer(_TickBuffer, DeltaTime, DeltaTime);
			CalculateNewValue(Error, DeltaTime);
			return true;
		}
		else if (FPIDController::AccumulateBuffer(_TickBuffer, DeltaTime, PeriodicDuration) == true)
		{
			// accumulate the periodic duration and calculate
			CalculateNewValue(Error, PeriodicDuration);
			return true;
		}


		// no calculations this frame
		return false;
	}

	// periodic duration is undefined
	// calculate on every frame
	CalculateNewValue(Error, DeltaTime);


	return true;

}


void FPIDController::SetPeriodicDuration(const float NewPeriodicDuration)
{
	if (NewPeriodicDuration > 0.f &&
		FPIDController::IsNearlyZero(NewPeriodicDuration) == false &&
		PeriodicDuration > 0.f &&
		FPIDController::IsNearlyZero(PeriodicDuration) == false)
	{
		const float GainChangeRatio = NewPeriodicDuration / PeriodicDuration;
		I_Gain *= GainChangeRatio;
		D_Gain /= GainChangeRatio;
	}

	PeriodicDuration = NewPeriodicDuration;


	return;
}


void FPIDController::SetEnabled(bool bIsEnabled, bool bClearIntegralAccumulation)
{
	if (IsEnabled() == false && bIsEnabled == true)
	{
		Initialize(bClearIntegralAccumulation);
	}

	_bIsEnabled = bIsEnabled;


	return;
}


void FPIDController::Initialize(bool bClearIntegralAccumulation)
{
	if (bClearIntegralAccumulation)
	{
		_IntegralAccumulation = 0.f;
	}
	else
	{
		_IntegralAccumulation = GetLastCalculatedValue();
	}

	// improvement -- clamp to prevent integral windup
	if (_IntegralAccumulation > ControlledValue_Max) _IntegralAccumulation = ControlledValue_Max;
	else if (_IntegralAccumulation < ControlledValue_Min) _IntegralAccumulation = ControlledValue_Min;

	ClearState();


	return;
}