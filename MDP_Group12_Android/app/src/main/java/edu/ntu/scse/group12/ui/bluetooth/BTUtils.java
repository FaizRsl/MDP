package edu.ntu.scse.group12.ui.bluetooth;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

public class BTUtils {
    public static final int REQUEST_BLUETOOTH_PERMISSIONS = 1;

    public static boolean checkBluetoothPermission(Context context) {
        int result = ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH);
        return result == PackageManager.PERMISSION_GRANTED;
    }

    public static boolean checkBluetoothAdminPermission(Context context) {
        int result = ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_ADMIN);
        return result == PackageManager.PERMISSION_GRANTED;
    }

    public static boolean checkLocationPermission(Context context) {
        int result = ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION);
        return result == PackageManager.PERMISSION_GRANTED;
    }

    public static boolean checkBluetoothConnectionPermission(Context context) {
        int result = ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT);
        return result == PackageManager.PERMISSION_GRANTED;
    }

    public static boolean checkBluetoothScanPermission(Context context) {
        int result = ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN);
        return result == PackageManager.PERMISSION_GRANTED;
    }

    public static void requestBluetoothPermissions(Activity activity) {
        // List of permissions to request
        String[] permissions = new String[]{
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_ADVERTISE,
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION
        };

        ActivityCompat.requestPermissions(activity, permissions, REQUEST_BLUETOOTH_PERMISSIONS);
    }
}