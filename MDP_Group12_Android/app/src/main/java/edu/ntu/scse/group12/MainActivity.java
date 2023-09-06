package edu.ntu.scse.group12;

import android.os.Bundle;
import android.util.Log;

import com.google.android.material.bottomnavigation.BottomNavigationView;

import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentManager;
import androidx.navigation.NavController;
import androidx.navigation.Navigation;
import androidx.navigation.fragment.NavHostFragment;
import androidx.navigation.ui.AppBarConfiguration;
import androidx.navigation.ui.NavigationUI;

import java.util.List;

import edu.ntu.scse.group12.databinding.ActivityMainBinding;
import edu.ntu.scse.group12.ui.bluetooth.BluetoothClient;
import edu.ntu.scse.group12.ui.home.HomeFragment;

public class MainActivity extends AppCompatActivity implements OnDataReceivedListener {

    private ActivityMainBinding binding;
    public static BluetoothClient bluetoothClient;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        BottomNavigationView navView = findViewById(R.id.nav_view);
        // Passing each menu ID as a set of Ids because each
        // menu should be considered as top level destinations.
        AppBarConfiguration appBarConfiguration = new AppBarConfiguration.Builder(
                R.id.navigation_home, R.id.navigation_bluetooth, R.id.navigation_notifications)
                .build();
        NavController navController = Navigation.findNavController(this, R.id.nav_host_fragment_activity_main);
        NavigationUI.setupActionBarWithNavController(this, navController, appBarConfiguration);
        NavigationUI.setupWithNavController(binding.navView, navController);
    }

    @Override
    public void onDataReceived(String data) {
        FragmentManager fragmentManager = getSupportFragmentManager();
        List<Fragment> fragments = fragmentManager.getFragments();
        if(fragments != null) {
            for (Fragment fragment : fragments) {
                if (fragment != null && fragment instanceof NavHostFragment) {
                    FragmentManager childFragmentManager = fragment.getChildFragmentManager();
                    List<Fragment> childFragments = childFragmentManager.getFragments();
                    if (childFragments != null) {
                        for (Fragment childFragment : childFragments) {
                            if (childFragment != null && childFragment.isVisible() && childFragment instanceof HomeFragment) {
                                ((HomeFragment) childFragment).updateReceivedData(data);
                                Log.i("CurrentFragment", "Data updated in HomeFragment");
                            }
                        }
                    }
                }
            }
        }
    }
}