import numpy as np
import joblib
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import (accuracy_score, classification_report,
                             confusion_matrix)
import time

print("=" * 50)
print("STEP 3a: RANDOM FOREST TRAINING")
print("=" * 50)

# ============================================================
# LOAD DATA
# ============================================================
print("\nLoading preprocessed data...")
X_train = np.load('RF_X_train.npy')
X_val   = np.load('RF_X_val.npy')
X_test  = np.load('RF_X_test.npy')
y_train = np.load('RF_y_train.npy')
y_val   = np.load('RF_y_val.npy')
y_test  = np.load('RF_y_test.npy')

le = joblib.load('label_encoder.joblib')

print(f"Training samples:   {len(X_train):,}")
print(f"Validation samples: {len(X_val):,}")
print(f"Test samples:       {len(X_test):,}")

# ============================================================
# TRAIN RANDOM FOREST
# ============================================================
print("\nTraining Random Forest...")
print("(This may take 2-5 minutes on your PC)")

start = time.time()

rf_model = RandomForestClassifier(
    n_estimators=100,
    max_depth=20,
    min_samples_leaf=5,
    random_state=42,
    n_jobs=-1,       # use all CPU cores
    verbose=1
)

rf_model.fit(X_train, y_train)
train_time = time.time() - start
print(f"\n✓ Training complete in {train_time:.1f} seconds")

# ============================================================
# EVALUATE
# ============================================================
print("\n--- VALIDATION SET ---")
y_val_pred = rf_model.predict(X_val)
val_acc = accuracy_score(y_val, y_val_pred)
print(f"Validation Accuracy: {val_acc*100:.2f}%")

print("\n--- TEST SET ---")
y_test_pred = rf_model.predict(X_test)
test_acc = accuracy_score(y_test, y_test_pred)
print(f"Test Accuracy: {test_acc*100:.2f}%")

print("\nClassification Report (Test Set):")
print(classification_report(y_test, y_test_pred,
                            target_names=le.classes_))

print("Confusion Matrix:")
cm = confusion_matrix(y_test, y_test_pred)
print(cm)
print("(Rows=Actual, Columns=Predicted)")
print(f"Classes: {list(le.classes_)}")

# ============================================================
# FEATURE IMPORTANCE
# ============================================================
feature_cols = [
    'supply_temp_C', 'room_temp_C', 'temp_differential_C',
    'vibration_magnitude', 'vibration_std', 'gyroscope',
    'current_A', 'low_pressure_PSI', 'compressor_on'
]

print("\nFeature Importance (top 10):")
importances = rf_model.feature_importances_
sorted_idx = importances.argsort()[::-1]
for i in sorted_idx:
    print(f"  {feature_cols[i]:<25} {importances[i]:.4f}")

# ============================================================
# INFERENCE SPEED TEST
# ============================================================
print("\nInference speed test (1,000 predictions)...")
start = time.time()
for _ in range(1000):
    rf_model.predict(X_test[:1])
inference_time = (time.time() - start) / 1000 * 1000
print(f"Average inference time: {inference_time:.2f}ms per sample")
print(f"Target: <500ms — {'✅ PASS' if inference_time < 500 else '❌ FAIL'}")

# ============================================================
# SAVE MODEL
# ============================================================
joblib.dump(rf_model, 'rf_model.joblib')
import os
model_size = os.path.getsize('rf_model.joblib') / (1024*1024)
print(f"\n✓ Model saved as rf_model.joblib ({model_size:.1f}MB)")
print("\nReady for Step 3b: LSTM Training")

