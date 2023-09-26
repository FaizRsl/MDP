package edu.ntu.scse.group12.ui.bluetooth;

import static android.app.Activity.RESULT_OK;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.ntu.scse.group12.MainActivity;
import edu.ntu.scse.group12.R;
import edu.ntu.scse.group12.databinding.FragmentBluetoothBinding;

public class BluetoothFragment extends Fragment {
    private final int LOCATION_PERMISSION_REQUEST = 101;
    private static final int REQUEST_ENABLE_BT = 1;

/*    private static final int REQUEST_PERMISSION_BLUETOOTH = 3;

    private static final int REQUEST_PERMISSION_BLUETOOTH_ADMIN = 4;*/
    private static final int REQUEST_PERMISSION_LOCATION = 2;

/*    private static final int REQUEST_PERMISSION_SCAN = 5;

    private static final int REQUEST_PERMISSION_CONNECT = 6;*/

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothClient bluetoothClient;
    ListView ll;
    ArrayAdapter<String> arrayAdapter;
    private List<String> deviceList = new ArrayList<>();
    final Set<String> devicesSet = new HashSet<>();
    private FragmentBluetoothBinding binding;

    private BluetoothDevice pairedDevice;

    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {
        BluetoothViewModel bluetoothViewModel = new ViewModelProvider(this, new ViewModelProvider.NewInstanceFactory()).get(BluetoothViewModel.class);

        binding = FragmentBluetoothBinding.inflate(inflater, container, false);
        View root = binding.getRoot();

        final TextView textView = binding.textBluetooth;
        bluetoothViewModel.getText().observe(getViewLifecycleOwner(), textView::setText);

        //start
        ll = (ListView) root.findViewById(R.id.listView);
        arrayAdapter = new ArrayAdapter<>(getContext(), android.R.layout.simple_list_item_1, deviceList);
        ll.setAdapter(arrayAdapter);


        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (bluetoothAdapter == null) {
            Log.d("Bluetooth", "Device doesn't support Bluetooth");
            Toast.makeText(getContext(), "Bluetooth is not supported in this device.", Toast.LENGTH_SHORT).show();
        }
        // Bluetooth is not enabled
        if (!bluetoothAdapter.isEnabled()) {
            Log.d("Bluetooth", "test if it runs here");
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }

        /*if (!BTUtils.checkBluetoothPermission(getContext())) {
            ActivityCompat.requestPermissions(requireActivity(), new String[]{Manifest.permission.BLUETOOTH}, REQUEST_PERMISSION_BLUETOOTH);
        }*/

        /*if (!BTUtils.checkBluetoothAdminPermission(getContext())) {
            ActivityCompat.requestPermissions(requireActivity(), new String[]{Manifest.permission.BLUETOOTH_ADMIN}, REQUEST_PERMISSION_BLUETOOTH_ADMIN);
        }*/

        if (!BTUtils.checkLocationPermission(getContext())) {
            ActivityCompat.requestPermissions(requireActivity(), new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, REQUEST_PERMISSION_LOCATION);
        }

        /*if (!BTUtils.checkBluetoothScanPermission(getContext())) {
            ActivityCompat.requestPermissions(requireActivity(), new String[]{Manifest.permission.BLUETOOTH_SCAN}, REQUEST_PERMISSION_SCAN);
        }*/

        /*if (!BTUtils.checkBluetoothConnectionPermission(getContext())) {
            ActivityCompat.requestPermissions(requireActivity(), new String[]{Manifest.permission.BLUETOOTH_CONNECT}, REQUEST_PERMISSION_CONNECT);
        }*/

        //On Bluetooth
        Button onBluetooth = root.findViewById(R.id.onBlueooth);
        onBluetooth.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Log.d("Bluetooth", "on click bluetooth...");
                if (bluetoothAdapter == null) {
                    Log.d("Bluetooth", "bluetoothAdapter is null...");
                    Toast.makeText(getContext(), "Bluetooth is not supported", Toast.LENGTH_SHORT).show();
                }
                else if(bluetoothAdapter.isEnabled()){
                    Log.d("Bluetooth", "bluetooth is enable");
                }
                else {
                    if (!bluetoothAdapter.isEnabled()) {
                        Log.d("Bluetooth", "enabling bluetooth...");
                        Intent i = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                        startActivityForResult(i, 1);
                    }
                }
            }
        });

        Button scanBluetooth = root.findViewById(R.id.scanBluetooth);
        scanBluetooth.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Log.d("Bluetooth", "Starting Bluetooth scan...");
                if (BTUtils.checkLocationPermission(getContext())){
                    Log.d("Bluetooth", "location enable");
                }
                if (BTUtils.checkBluetoothPermission(getContext())){
                    Log.d("Bluetooth", "bluetooth enable");
                }
                /*if (BTUtils.checkBluetoothAdminPermission(getContext())){
                    Log.d("Bluetooth", "bluetooth admin enable");
                }
                if (BTUtils.checkBluetoothConnectionPermission(getContext())){
                    Log.d("Bluetooth", "bluetooth connection enable");
                }
                else{
                    ActivityCompat.requestPermissions(requireActivity(), new String[]{Manifest.permission.BLUETOOTH_CONNECT}, REQUEST_PERMISSION_CONNECT);
                }
                if (BTUtils.checkBluetoothScanPermission(getContext())){
                    Log.d("Bluetooth", "bluetooth scan enable");
                }*/

                // Check if we have the location permission before starting discovery
                if (BTUtils.checkLocationPermission(getContext())) {
                    boolean discoveryStarted = bluetoothAdapter.startDiscovery();
                    if (discoveryStarted) {
                        Log.d("Bluetooth", "Bluetooth scan started...");
                        Toast.makeText(getContext(), "Bluetooth scan started...", Toast.LENGTH_SHORT).show();
                        // Stop discovery after 60 sec
                        //new Handler().postDelayed(new Runnable() {
                            //public void run() {
                              //  if (BTUtils.checkBluetoothScanPermission(getContext())) {
                               //     if (bluetoothAdapter.isDiscovering()) {
                                //        bluetoothAdapter.cancelDiscovery();
                               //         Log.d("Bluetooth", "Bluetooth scan stopped after 1 minute.");
                               //     }
                              //  }else{
                             //       BTUtils.requestBluetoothPermissions(getActivity());
                              //  }
                           // }
                        //}, 60000);
                    } else {
                        Log.d("Bluetooth", "Could not start Bluetooth scan.");
                        Toast.makeText(getContext(), "Could not start Bluetooth scan.", Toast.LENGTH_SHORT).show();
                    }
                } else {
                    Log.d("Bluetooth", "Failed to start Bluetooth scan. Location permission not granted.");
                    BTUtils.requestBluetoothPermissions(getActivity());
                }
            }
        });

        IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
        requireActivity().registerReceiver(receiver, filter);

    // Set an item click listener for your ListView
        ll.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {

                if(BTUtils.checkBluetoothConnectionPermission(getContext())){
                    Log.d("permission","permission granted");
                }else{
                    BTUtils.requestBluetoothPermissions(getActivity());
                }
                // Get the device MAC address which is the last 17 chars in the View
                String info = ((TextView) view).getText().toString();
                String address = info.substring(info.length() - 17);
                //Log.d("address", "address >>> " + address);
                BluetoothDevice device = bluetoothAdapter.getRemoteDevice(address);
                pairedDevice = bluetoothAdapter.getRemoteDevice(String.valueOf(device));

                if (device.getBondState() != BluetoothDevice.BOND_BONDED) {
                    Toast.makeText(getContext(), "Pairing with device...", Toast.LENGTH_SHORT).show();
                    device.createBond();
                    IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
                    getContext().registerReceiver(new BroadcastReceiver() {
                        @Override
                        public void onReceive(Context context, Intent intent) {
                            int state = intent.getIntExtra(BluetoothDevice.EXTRA_BOND_STATE, BluetoothDevice.ERROR);
                            if (state == BluetoothDevice.BOND_BONDED) {
                                //Toast.makeText(getContext(), "Device paired. Connecting...", Toast.LENGTH_SHORT).show();
                                checkBluetoothPermissions(address);
                                context.unregisterReceiver(this);
                            }
                        }
                    }, filter);
                } if (device.getBondState() == BluetoothDevice.BOND_BONDED) {

                    if(bluetoothAdapter.isDiscovering()){
                        //cancel scanning
                        Log.d("cancel","cancelling Discovery...");
                        bluetoothAdapter.cancelDiscovery();
                    }
                    Toast.makeText(getContext(), "Device already paired...", Toast.LENGTH_SHORT).show();
                    Log.d("pairedDevice","pairedDevice >>> " +pairedDevice);
                    Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices();
                    if (pairedDevices.contains(pairedDevice)) {
                        Log.d("pairedDevice","Device is paired...");
                    }
                    checkBluetoothPermissions(address);
                }
            }
        });

        return root;
    }

    private final BroadcastReceiver receiver = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                // Discovery has found a device. Get the BluetoothDevice object and its info from the Intent
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);

                // Check if we have the location permission before getting the device name
                if (BTUtils.checkLocationPermission(getContext())) {
                    String deviceName = device.getName();
                    String deviceHardwareAddress = device.getAddress(); // MAC address

                    // Get the current timestamp
                    String timeStamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date());

                    if (deviceName != null && deviceHardwareAddress != null) {
                        String deviceInfo = "Device found: " + deviceName + ", MAC: " + deviceHardwareAddress;
                        devicesSet.add(deviceInfo);
                        List<String> devicesList = new ArrayList<>(devicesSet);
                        for (int i = 0; i < devicesList.size(); i++) {
                            Log.d("Bluetooth", "Device found: " + deviceName + ", MAC: " + deviceHardwareAddress + ", Timestamp: " + timeStamp);
                            devicesList.set(i, (i + 1) + ". " + devicesList.get(i));
                        }
                        ArrayAdapter<String> adapter = new ArrayAdapter<>(getContext(), android.R.layout.simple_list_item_1, devicesList);
                        ll.setAdapter(adapter);
                    }
                } else {
                    Log.d("Bluetooth", "Failed to get device name. Location permission not granted.");
                }
            }
        }
    };

    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        Log.d("onActivityResult", "calling onActivityResult()");
        if (requestCode == 1) {
            if (resultCode == RESULT_OK) {
                Toast.makeText(getContext(), "Bluetooth Enabled.", Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(getContext(), "Bluetooth not enabled. App functionality may be limited.", Toast.LENGTH_SHORT).show();
            }
        }
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        getActivity().unregisterReceiver(receiver);
        if (bluetoothClient != null) {
            bluetoothClient.stopClient();
        }
        binding = null;
    }

    private void checkBluetoothPermissions(String pairedDevice) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if (getContext().checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, LOCATION_PERMISSION_REQUEST);
            } else {
                connectToBluetoothDevice(pairedDevice);
            }
        } else {
            connectToBluetoothDevice(pairedDevice);
        }
    }
    private void connectToBluetoothDevice(String pairedDevice) {
        MainActivity.bluetoothClient = new BluetoothClient(pairedDevice, getContext());
        MainActivity.bluetoothClient.setOnDataReceivedListener((MainActivity)getActivity());
        MainActivity.bluetoothClient.start();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        if (requestCode == LOCATION_PERMISSION_REQUEST) {
            if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // Permission granted, retry the connection
                if (bluetoothClient != null) {
                    bluetoothClient.start();
                }
            } else {
                Toast.makeText(getContext(), "Bluetooth permissions not granted", Toast.LENGTH_SHORT).show();
            }
        }
    }
}