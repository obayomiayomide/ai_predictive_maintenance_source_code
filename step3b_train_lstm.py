import numpy as np
import joblib
import time
import os

print("=" * 50)
print("STEP 3b: LSTM TRAINING")
print("=" * 50)

# ============================================================
# LOAD DATA
# ============================================================
print("\nLoading LSTM sequences...")
X_train = np.load('LSTM_X_train.npy')
X_val   = np.load('LSTM_X_val.npy')
X_test  = np.load('LSTM_X_test.npy')
y_train = np.load('LSTM_y_train.npy')
y_val   = np.load('LSTM_y_val.npy')
y_test  = np.load('LSTM_y_test.npy')
le      = joblib.load('label_encoder.joblib')

print(f"Train shape: {X_train.shape}")
print(f"Val shape:   {X_val.shape}")
print(f"Test shape:  {X_test.shape}")
print(f"Classes:     {list(le.classes_)}")

# ============================================================
# BUILD LSTM MODEL
# ============================================================
try:
    import tensorflow as tf
    from tensorflow.keras.models import Sequential
    from tensorflow.keras.layers import (LSTM, Dense, Dropout,
                                         Bidirectional)
    from tensorflow.keras.callbacks import (EarlyStopping,
                                            ModelCheckpoint)
    from tensorflow.keras.utils import to_categorical

    print(f"\nTensorFlow version: {tf.__version__}")

    NUM_CLASSES = len(le.classes_)

    # One-hot encode labels
    y_train_cat = to_categorical(y_train, NUM_CLASSES)
    y_val_cat   = to_categorical(y_val,   NUM_CLASSES)
    y_test_cat  = to_categorical(y_test,  NUM_CLASSES)

    # Build model
    model = Sequential([
        Bidirectional(LSTM(64, return_sequences=True),
                      input_shape=(X_train.shape[1], X_train.shape[2])),
        Dropout(0.3),
        LSTM(32),
        Dropout(0.3),
        Dense(16, activation='relu'),
        Dropout(0.2),
        Dense(NUM_CLASSES, activation='softmax')
    ])

    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
        loss='categorical_crossentropy',
        metrics=['accuracy']
    )

    model.summary()

    # Callbacks
    callbacks = [
        EarlyStopping(monitor='val_loss', patience=10,
                      restore_best_weights=True, verbose=1),
        ModelCheckpoint('lstm_best.h5', monitor='val_accuracy',
                        save_best_only=True, verbose=1)
    ]

    # Train
    print("\nTraining LSTM...")
    print("(This may take 20-60 mins on your PC — go grab a coffee!)")
    start = time.time()

    history = model.fit(
        X_train, y_train_cat,
        validation_data=(X_val, y_val_cat),
        epochs=50,
        batch_size=32,
        callbacks=callbacks,
        verbose=1
    )

    train_time = time.time() - start
    print(f"\n✓ Training complete in {train_time/60:.1f} minutes")

    # ============================================================
    # EVALUATE
    # ============================================================
    from sklearn.metrics import (accuracy_score, classification_report,
                                 confusion_matrix)

    y_test_pred_proba = model.predict(X_test)
    y_test_pred = np.argmax(y_test_pred_proba, axis=1)

    test_acc = accuracy_score(y_test, y_test_pred)
    print(f"\nTest Accuracy: {test_acc*100:.2f}%")
    print(f"Target: >90% — {'✅ PASS' if test_acc > 0.90 else '❌ FAIL'}")

    print("\nClassification Report:")
    print(classification_report(y_test, y_test_pred,
                                target_names=le.classes_))

    print("Confusion Matrix:")
    print(confusion_matrix(y_test, y_test_pred))

    # Inference speed
    print("\nInference speed test...")
    start = time.time()
    for _ in range(100):
        model.predict(X_test[:1], verbose=0)
    inf_time = (time.time() - start) / 100 * 1000
    print(f"Average inference time: {inf_time:.1f}ms")
    print(f"Target: <500ms — {'✅ PASS' if inf_time < 500 else '❌ FAIL'}")

    # ============================================================
    # SAVE MODELS
    # ============================================================
    # Save full model
    model.save('lstm_model.h5')
    size_h5 = os.path.getsize('lstm_model.h5') / (1024*1024)
    print(f"\n✓ Full model saved: lstm_model.h5 ({size_h5:.1f}MB)")

    # Convert to TFLite
    print("\nConverting to TFLite (float16 quantization)...")
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.target_spec.supported_types = [tf.float16]
    # FIX: Bidirectional(LSTM(...)) produces dynamic-shape TensorListReserve
    # ops that the default TFLite converter cannot lower. Enabling
    # SELECT_TF_OPS allows the converter to fall back to native TF kernels
    # for those specific ops instead of failing the whole conversion.
    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS,
        tf.lite.OpsSet.SELECT_TF_OPS
    ]
    converter._experimental_lower_tensor_list_ops = False
    tflite_model = converter.convert()

    with open('lstm_model.tflite', 'wb') as f:
        f.write(tflite_model)

    size_tflite = os.path.getsize('lstm_model.tflite') / (1024*1024)
    print(f"✓ TFLite model saved: lstm_model.tflite ({size_tflite:.1f}MB)")
    print(f"  Size reduction: {(1 - size_tflite/size_h5)*100:.0f}%")

    print("\n" + "=" * 50)
    print("LSTM TRAINING COMPLETE")
    print("=" * 50)
    print("\nFiles saved:")
    print("  lstm_best.h5      — best checkpoint during training")
    print("  lstm_model.h5     — full trained model")
    print("  lstm_model.tflite — quantized model for deployment")
    print("\nReady for Step 4: Local Inference Testing")

except ImportError:
    print("\n⚠️  TensorFlow not installed.")
    print("Install it on your PC with:")
    print("  pip install tensorflow")
    print("\nThen run this script again.")
    print("Note: TF install is ~500MB — normal for ML work.")

