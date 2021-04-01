try:
    from .real_manual_evaluation_runtime import RealManualEvaluationRuntime
except Exception as e:
    print(f"RealManualEvaluationRuntime is disabled - {e}")
