package edu.ntu.scse.group12.ui.home;

import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.graphics.Rect;
import android.os.Bundle;
import android.text.TextUtils;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.GridLayout;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AlertDialog;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

import edu.ntu.scse.group12.MainActivity;
import edu.ntu.scse.group12.R;
import edu.ntu.scse.group12.ui.home.modal.Robot;
import edu.ntu.scse.group12.ui.home.modal.Obstacle;

public class HomeFragment extends Fragment {

    private GridLayout gridLayout;
    private int gridSize = 20, obstaclesSize = 0, addobstaclesSize = 0;
    private ImageView[][] imageViews;
    private ImageView robotImageView;
    private boolean placeCarActive = false, addObstacleActive = false, removeObstacleActive = false,
            dragObstacleActive = false, rotateObstacleActive = false, setUpButtonActive = false;
    private EditText receivedDataTextArea, statusTextArea;
    private static final String TAG = "HomeFragment";
    private ArrayList<Obstacle> obstacles;
    private Obstacle currentObstacle = null;
    private Robot myRobot;
    private FrameLayout[][] frameLayouts;

    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        View root = inflater.inflate(R.layout.fragment_home, container, false);
        final EditText sendDataTextArea = root.findViewById(R.id.sendDataTextArea);
        final LinearLayout homeLayout = root.findViewById(R.id.home_layout);
        gridLayout = root.findViewById(R.id.gridLayout);
        final Button addObstacleButton = root.findViewById(R.id.addObstacleButton);
        final Button removeButton = root.findViewById(R.id.removeButton);
        final Button placeCarButton = root.findViewById(R.id.placeCarButton);
        final Button rotateButton = root.findViewById(R.id.rotateButton);
        final Button sendDataButton = root.findViewById(R.id.sendDataButton);
        final Button dragObstacleButton = root.findViewById(R.id.dragObstacleButton);
        final ImageView upArrowButton = root.findViewById(R.id.upArrow);
        final ImageView downArrowButton = root.findViewById(R.id.downArrow);
        //final ImageView leftArrowButton = root.findViewById(R.id.leftArrow);
        //final ImageView rightArrowButton = root.findViewById(R.id.rightArrow);
        final Button captureButton = root.findViewById(R.id.btnRound);
        final ImageView upRightArrowButton = root.findViewById(R.id.upRightArrow);
        final ImageView upleftArrowButton = root.findViewById(R.id.upleftArrow);
        final ImageView downRightArrowButton = root.findViewById(R.id.downRightArrow);
        final ImageView downLeftArrowButton = root.findViewById(R.id.downLeftArrow);
        final Button setUpButton = root.findViewById(R.id.setUpButton);

        receivedDataTextArea = root.findViewById(R.id.receivedDataTextArea);
        statusTextArea = root.findViewById(R.id.robotStatusTextArea);

        obstacles = new ArrayList<>();
        imageViews = new ImageView[gridSize][gridSize];
        frameLayouts = new FrameLayout[gridSize][gridSize];
        gridLayout.setColumnCount(gridSize);
        gridLayout.setRowCount(gridSize);

        for (int j = 0; j < gridSize; j++) {
            for (int i = gridSize - 1; i >= 0; i--) {
                //ImageView imageView = new ImageView(getContext());
                FrameLayout frameLayout = new FrameLayout(getContext());
                GridLayout.LayoutParams params = new GridLayout.LayoutParams();
                params.width = 0;
                params.height = 0;
                params.columnSpec = GridLayout.spec(j, 1, 1f);
                params.rowSpec = GridLayout.spec(i, 1, 1f);
                frameLayout.setLayoutParams(params);
                frameLayout.setBackgroundColor(Color.WHITE);

                ImageView imageView = new ImageView(getContext());
                imageView.setPadding(0, 1, 0, 1);
                imageView.setBackground(ContextCompat.getDrawable(getContext(), R.drawable.cell_border));

                TextView textView = new TextView(getContext());
                textView.setLayoutParams(new FrameLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT, Gravity.CENTER));
                textView.setVisibility(View.INVISIBLE);
                frameLayout.addView(imageView);
                frameLayout.addView(textView);

                frameLayouts[j][gridSize - 1 - i] = frameLayout;
                gridLayout.addView(frameLayout);
                imageViews[j][gridSize - 1 - i] = imageView;
            }
        }

        addObstacleButton.setOnClickListener(view -> {
            if(!removeObstacleActive && !setUpButtonActive && !dragObstacleActive && !placeCarActive && !rotateObstacleActive){
                addObstacleActive = !addObstacleActive;
                updateButtonState(addObstacleButton, addObstacleActive);
            }else{
                Toast.makeText(getContext(), "Only one button is allowed to be active.", Toast.LENGTH_SHORT).show();
            }

        });

        dragObstacleButton.setOnClickListener(view -> {
            if(!removeObstacleActive && !setUpButtonActive && !addObstacleActive && !placeCarActive && !rotateObstacleActive) {
                dragObstacleActive = !dragObstacleActive;
                updateButtonState(dragObstacleButton, dragObstacleActive);
            }else{
                Toast.makeText(getContext(), "Only one button is allowed to be active.", Toast.LENGTH_SHORT).show();
            }
        });

        removeButton.setOnClickListener(view -> {
            if(!addObstacleActive && !setUpButtonActive && !dragObstacleActive && !placeCarActive && !rotateObstacleActive) {
                removeObstacleActive = !removeObstacleActive;
                updateButtonState(removeButton, removeObstacleActive);
            }else{
                Toast.makeText(getContext(), "Only one button is allowed to be active.", Toast.LENGTH_SHORT).show();
            }
        });

        rotateButton.setOnClickListener(view -> {
            if(!removeObstacleActive && !setUpButtonActive && !addObstacleActive && !dragObstacleActive && !placeCarActive){
                rotateObstacleActive = !rotateObstacleActive;
                updateButtonState(rotateButton, rotateObstacleActive);
            }else{
                Toast.makeText(getContext(), "Only one button is allowed to be active.", Toast.LENGTH_SHORT).show();
            }
        });

        placeCarButton.setOnClickListener(view -> {
            if (!removeObstacleActive && !setUpButtonActive && !addObstacleActive && !dragObstacleActive && !rotateObstacleActive) {
                placeCarActive = !placeCarActive;
                updateButtonState(placeCarButton, placeCarActive);
            } else {
                Toast.makeText(getContext(), "Only one button is allowed to be active.", Toast.LENGTH_SHORT).show();
            }
        });

        setUpButton.setOnClickListener(view -> {
            if (!removeObstacleActive && !placeCarActive && !addObstacleActive && !dragObstacleActive && !rotateObstacleActive) {
                setUpButtonActive = !setUpButtonActive;
                    showCustomDialog();
            } else {
                Toast.makeText(getContext(), "Only one button is allowed to be active.", Toast.LENGTH_SHORT).show();
            }
        });

        upArrowButton.setOnClickListener(view -> {
            moveRobotUp();
        });
        downArrowButton.setOnClickListener(view -> {
            moveRobotDown();
        });
        upRightArrowButton.setOnClickListener(view -> {
            moveRobotUpRight();
        });
        upleftArrowButton.setOnClickListener(view -> {
            moveRobotUpLeft();
        });
        downRightArrowButton.setOnClickListener(view -> {
            moveRobotBottomRight();
        });
        downLeftArrowButton.setOnClickListener(view -> {
            moveRobotBottomLeft();
        });

        captureButton.setOnClickListener(view -> {
            if(MainActivity.bluetoothClient != null) {
                MainActivity.bluetoothClient.sendData("SN");
            }
        });
        /*
        leftArrowButton.setOnClickListener(view -> {
            if(MainActivity.bluetoothClient != null){
                MainActivity.bluetoothClient.sendData("Arrow Left");
            }
            if(myRobot !=null) {
                moveRobotLeft();
            }
        });

        rightArrowButton.setOnClickListener(view -> {
            if(MainActivity.bluetoothClient != null){
                MainActivity.bluetoothClient.sendData("Arrow Right");
            }
            if(myRobot !=null){
                moveRobotRight();
            }
        });
        */


        gridLayout.setOnTouchListener((view, motionEvent) -> {
            int row = Math.min(gridSize - 1, (int) (motionEvent.getX() / view.getWidth() * gridSize));
            int col = Math.min(gridSize - 1, gridSize - 1 - (int) (motionEvent.getY() / view.getHeight() * gridSize));
            //drag up of the grid and remove
            if(col<0){
                switch (motionEvent.getAction()){
                    case MotionEvent.ACTION_MOVE:
                        if (dragObstacleActive) {
                            Log.d(TAG, "CURRENT OBS: " +currentObstacle);
                            if(currentObstacle!=null){
                                removeObstacle(currentObstacle.getRow(),currentObstacle.getCol());
                            }
                        }
                        return true;
                }
            }

            if(row >= 0 && col >= 0) {
                switch (motionEvent.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (addObstacleActive) {
                            if (isCellOccupiedByRobot(row, col)) {
                                Toast.makeText(getContext(), "This cell is occupied by the robot.", Toast.LENGTH_SHORT).show();
                                return true;
                            }
                            if(obstacles.size()>0){
                                for (Obstacle obstacle : obstacles){
                                    if(obstacle.getRow() == row && obstacle.getCol() == col){
                                        Toast.makeText(getContext(), "This cell is occupied by an obstacle.", Toast.LENGTH_SHORT).show();
                                        return true;
                                    }
                                }
                            }

                            Obstacle obstacle = new Obstacle(row, col, addobstaclesSize, Obstacle.NORTH);
                            obstacles.add(obstacle);
                            addobstaclesSize = addobstaclesSize + 1;

                            imageViews[obstacle.getRow()][obstacle.getCol()].setImageResource(getDrawableForDirection("obstacle", obstacle.getDirection()));

                            FrameLayout frameLayout = frameLayouts[obstacle.getRow()][obstacle.getCol()];
                            TextView textView = (TextView) frameLayout.getChildAt(1);  // get the TextView

                            if(obstacle != null && obstacle.getImageId() != 0){
                                textView.setText(String.valueOf(obstacle.getImageId()));
                            }else{
                                textView.setText(String.valueOf(obstacle.getId()));  // set the text
                            }

                            textView.setTextColor(Color.WHITE);
                            textView.setVisibility(View.VISIBLE);  // make it visible
                            if(MainActivity.bluetoothClient != null){
                                MainActivity.bluetoothClient.sendData("Obstacle: " +obstacle.toString());
                            }
                            return true;
                        } else if (removeObstacleActive) {
                            removeObstacle(row, col);
                            return true;
                        } else if (dragObstacleActive) {
                            Obstacle obstacle = getObstacleAt(row, col);
                            if (obstacle != null) {
                                currentObstacle = obstacle;
                                currentObstacle.isBeingDragged = true;
                                if(MainActivity.bluetoothClient != null){
                                    MainActivity.bluetoothClient.sendData("Drag Obstacle: " +currentObstacle.toString());
                                }
                            }
                            return true;
                        }else if (rotateObstacleActive) {
                            Obstacle obstacle = getObstacleAt(row, col);
                            if (obstacle != null) {
                                rotateObstacle(obstacle);
                                if(MainActivity.bluetoothClient != null){
                                    MainActivity.bluetoothClient.sendData("rotate obs: " +obstacle.toString());
                                }
                            }
                            if (myRobot != null && row >= myRobot.getRow() - 1 && row <= myRobot.getRow() + 1
                                    && col >= myRobot.getCol() - 1 && col <= myRobot.getCol() + 1) {
                                rotateRobot();
                                if(MainActivity.bluetoothClient != null){
                                    MainActivity.bluetoothClient.sendData("myRobot rotate >>> " + myRobot.toString());
                                }
                                return true;
                            }
                            return true;
                        } else if (placeCarActive) {
                            placeRobot(row, col, 0);//set default to north
                            return true;
                        }

                    case MotionEvent.ACTION_MOVE:
                        if (dragObstacleActive && currentObstacle != null) {
                            if (isCellOccupiedByRobot(row, col)) {
                                Toast.makeText(getContext(), "This cell is occupied by the robot.", Toast.LENGTH_SHORT).show();
                                return true;
                            }
                            Log.d(TAG,"current Obstacle: " +currentObstacle.toString());
                            if(MainActivity.bluetoothClient != null){
                                MainActivity.bluetoothClient.sendData("Obstacle dragged to: " +currentObstacle.toString());
                            }
                            // remove previous obstacle image and hide associated text view
                            FrameLayout oldFrameLayout = frameLayouts[currentObstacle.getRow()][currentObstacle.getCol()];
                            ImageView oldImageView = (ImageView) oldFrameLayout.getChildAt(0);  // get the ImageView
                            oldImageView.setImageResource(0);  // remove the image
                            TextView oldTextView = (TextView) oldFrameLayout.getChildAt(1);  // get the TextView
                            oldTextView.setVisibility(View.INVISIBLE);  // make it invisible

                            // check if new cell is occupied by another obstacle
                            Obstacle nextObstacle = getObstacleAt(row, col);
                            if(nextObstacle == null || nextObstacle == currentObstacle) {
                                currentObstacle.setRow(row);
                                currentObstacle.setCol(col);
                                if(isOutsideGrid(currentObstacle)) {
                                    removeObstacle(currentObstacle.getRow(), currentObstacle.getCol());
                                    Toast.makeText(getContext(), "Obstacle removed as it was dragged outside", Toast.LENGTH_SHORT).show();
                                    currentObstacle = null;
                                } else {
                                    FrameLayout newFrameLayout = frameLayouts[currentObstacle.getRow()][currentObstacle.getCol()];
                                    ImageView newImageView = (ImageView) newFrameLayout.getChildAt(0);  // get the ImageView
                                    newImageView.setImageResource(getDrawableForDirection("obstacle",currentObstacle.getDirection()));  // set the image

                                    TextView newTextView = (TextView) newFrameLayout.getChildAt(1);  // get the TextView

                                    if(currentObstacle != null && currentObstacle.getImageId() != 0){
                                        newTextView.setText(String.valueOf(currentObstacle.getImageId()));  // set the text
                                    }else{
                                        newTextView.setText(String.valueOf(currentObstacle.getId()));  // set the text
                                    }

                                    newTextView.setTextColor(Color.WHITE);
                                    newTextView.setVisibility(View.VISIBLE);  // make it visible
                                }
                            } else {
                                Toast.makeText(getContext(), "This cell is occupied by another obstacle.", Toast.LENGTH_SHORT).show();
                            }
                            return true;
                        }
                    case MotionEvent.ACTION_UP:
                    case MotionEvent.ACTION_CANCEL:
                        if (dragObstacleActive && currentObstacle != null) {
                            if (isOutsideGrid(currentObstacle)) {
                                Log.d("ccc",currentObstacle.toString());
                                removeObstacle(currentObstacle.getRow(), currentObstacle.getCol());
                                currentObstacle = null;
                                Toast.makeText(getContext(), "Obstacle removed as it was released outside", Toast.LENGTH_SHORT).show();
                            }
                            if (currentObstacle != null) {
                                currentObstacle.isBeingDragged = false;
                                currentObstacle = null;
                            }
                        }
                        Log.i(TAG, "Last position: (" + row + "," + col + ")");
                        if(myRobot != null){
                            Log.i("ACTION_UP", "Robot: " + myRobot.toString());
                        }
                        if(obstacles != null){
                            for(int i=0;i<obstacles.size();i++){
                                Log.i(TAG, "Obstacles: "+ i + " >>> " +obstacles.get(i).toString());
                            }

                        }

                        return true;
                }
            }
            return false;
        });

        homeLayout.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (sendDataTextArea.isFocused()) {
                    Rect outRect = new Rect();
                    sendDataTextArea.getGlobalVisibleRect(outRect);
                    if (!outRect.contains((int)event.getRawX(), (int)event.getRawY())) {
                        sendDataTextArea.clearFocus();
                        hideKeyboardFrom(getContext(), v);
                    }
                }
                return false;
            }
        });

        sendDataButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String dataToSend = sendDataTextArea.getText().toString();
                if (!dataToSend.isEmpty() && MainActivity.bluetoothClient != null) {
                    if(dataToSend.toLowerCase().startsWith("start")){
                        //start{obstacles:[{x:1,y:1,d:0,id:1},{x:2,y:2,d:2,id:2},{x:3,y:3,d:4,id:3},{x:4,y:4,d:6,id:4},{x:5,y:5,d:0,id:5}],robot_row:10,robot_col:10,robot_dir:0}
                        MainActivity.bluetoothClient.sendData(createSetUpJson(obstacles,myRobot));
                    }else {
                        MainActivity.bluetoothClient.sendData(dataToSend);
                    }
                    Toast.makeText(getContext(), "Send Data Success...", Toast.LENGTH_SHORT).show();
                    sendDataTextArea.getText().clear();
                }else{
                    Toast.makeText(getContext(), "Send Data Failed...", Toast.LENGTH_SHORT).show();
                }
            }
        });

        return root;
    }
    private void showCustomDialog(){
        LayoutInflater inflater = getLayoutInflater();
        View dialogView = inflater.inflate(R.layout.dialog_layout, null);

        AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(getContext());
        alertDialogBuilder.setView(dialogView);
        AlertDialog alertDialog = alertDialogBuilder.create();

        Button addButton = dialogView.findViewById(R.id.add_obstacle);

        LinearLayout obstacleInputs = dialogView.findViewById(R.id.obstacle_inputs);

        // Add initial set of Obstacle inputs
        View initialObstacleInput = inflater.inflate(R.layout.obstacle_input, null);
        obstacleInputs.addView(initialObstacleInput);

        addButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                View obstacleInput = getLayoutInflater().inflate(R.layout.obstacle_input, null);
                obstacleInputs.addView(obstacleInput);
            }
        });

        EditText rowRobot = dialogView.findViewById(R.id.row_robot);
        EditText colRobot = dialogView.findViewById(R.id.col_robot);
        EditText dirRobot = dialogView.findViewById(R.id.dir_robot);
        Button submitButton = dialogView.findViewById(R.id.submit);
        submitButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(obstacles.size() > 0) {
                    List<Obstacle> tempObstacles = new ArrayList<>(obstacles);
                    for(Obstacle obs: tempObstacles) {
                        removeObstacle(obs.getRow(), obs.getCol());
                    }
                }

                myRobot = null;
                for (int i = 0; i < obstacleInputs.getChildCount(); i++) {
                    View obstacleInput = obstacleInputs.getChildAt(i);
                    EditText rowObstacle = obstacleInput.findViewById(R.id.row_obstacle_input);
                    EditText colObstacle = obstacleInput.findViewById(R.id.col_obstacle_input);
                    EditText dirObstacle = obstacleInput.findViewById(R.id.dir_obstacle_input);

                    if (TextUtils.isEmpty(rowObstacle.getText().toString()) || TextUtils.isEmpty(colObstacle.getText().toString())
                            || TextUtils.isEmpty(dirObstacle.getText().toString())) {
                        Toast.makeText(getContext(), "Obstacle input is empty...", Toast.LENGTH_SHORT).show();
                        return;
                    }

                    if(!validateSetUpCoordinate(rowObstacle) || !validateSetUpCoordinate(colObstacle)){
                        Toast.makeText(getContext(), "Invalid Obstacle coordinates value...", Toast.LENGTH_SHORT).show();
                        return;
                    }

                    if (!validateSetUpDirection(dirObstacle)) {
                        Toast.makeText(getContext(), "Invalid Obstacle direction value...", Toast.LENGTH_SHORT).show();
                        return;
                    }

                    if (!validateSetUpDirection(dirRobot)) {
                        Toast.makeText(getContext(), "Invalid Robot direction value...", Toast.LENGTH_SHORT).show();
                        return;
                    }

                    if(!validateSetUpCoordinate(rowRobot) || !validateSetUpCoordinate(colRobot)){
                        Toast.makeText(getContext(), "Invalid Robot coordinates value...", Toast.LENGTH_SHORT).show();
                        return;
                    }

                    Obstacle obstacle = new Obstacle(
                            Integer.parseInt(rowObstacle.getText().toString()),
                            Integer.parseInt(colObstacle.getText().toString()),
                            i,
                            Integer.parseInt(dirObstacle.getText().toString())
                    );
                    obstacles.add(obstacle);
                    obstaclesSize = i; // assign current value

                    FrameLayout frameLayout = frameLayouts[obstacle.getRow()][obstacle.getCol()];
                    imageViews[obstacle.getRow()][obstacle.getCol()].setImageResource(getDrawableForDirection("obstacle", obstacle.getDirection()));

                    TextView textView = (TextView) frameLayout.getChildAt(1);  // get the TextView

                    if(obstacle != null && obstacle.getImageId() != 0){
                        textView.setText(String.valueOf(obstacle.getImageId()));
                    }else{
                        textView.setText(String.valueOf(obstacle.getId()));  // set the text
                    }

                    textView.setTextColor(Color.WHITE);
                    textView.setVisibility(View.VISIBLE);  // make it visible
                }
                addobstaclesSize = obstaclesSize +1;
                if (TextUtils.isEmpty(rowRobot.getText().toString()) || TextUtils.isEmpty(colRobot.getText().toString())
                        || TextUtils.isEmpty(dirRobot.getText().toString())) {
                    Toast.makeText(getContext(), "Robot input is empty...", Toast.LENGTH_SHORT).show();
                    return;
                }
                myRobot = new Robot(
                        Integer.parseInt(rowRobot.getText().toString()),
                        Integer.parseInt(colRobot.getText().toString()),
                        Integer.parseInt(dirRobot.getText().toString())
                );
                placeRobot(myRobot.getRow(), myRobot.getCol(), myRobot.getDirection());
                alertDialog.dismiss();
                if(obstacles != null){
                    for(int i=0;i<obstacles.size();i++){
                        Log.i("Dialog", "Obstacles: "+ i + " >>> " +obstacles.get(i).toString());
                    }
                }
                if(myRobot != null){
                    Log.d("Dialog Box","Robot; "+ myRobot.toString());
                }
            }
        });
        setUpButtonActive = false;
        alertDialog.show();
    }


    private boolean isOutsideGrid(Obstacle obstacle) {
        Log.d("ccc","outside....");
        return obstacle.getRow() < 0 || obstacle.getCol() < 0 || gridSize - 1 - obstacle.getRow() >= gridSize || obstacle.getCol() >= gridSize;
    }
    private Obstacle getObstacleAt(int row, int col) {
        for (Obstacle obstacle : obstacles) {
            if (obstacle.getRow() == row && obstacle.getCol() == col) {
                return obstacle;
            }
        }
        return null;
    }

    private void removeObstacle(int row, int col) {
        Obstacle obstacle = getObstacleAt(row, col);
        Log.d(TAG,"checking..."+getObstacleAt(row, col));
        if (obstacle != null) {
            obstacles.remove(obstacle);
            imageViews[obstacle.getRow()][obstacle.getCol()].setImageResource(0); //remove
            FrameLayout frameLayout = frameLayouts[obstacle.getRow()][obstacle.getCol()];
            TextView textView = (TextView) frameLayout.getChildAt(1);  // get the TextView
            textView.setVisibility(View.INVISIBLE);  // make it invisible
            currentObstacle = null;
        }
        if (myRobot != null && row >= myRobot.getRow() - 1 && row <= myRobot.getRow() + 1
                && col >= myRobot.getCol() - 1 && col <= myRobot.getCol() + 1) {
            if (robotImageView != null) {
                gridLayout.removeView(robotImageView);
                myRobot = null;
            }
        }
    }

    private int getDrawableForDirection(String obj, int direction) {
        if(obj.equalsIgnoreCase("obstacle")){
            switch (direction) {
                case Obstacle.NORTH:
                    return R.drawable.obstacle_north;
                case Obstacle.EAST:
                    return R.drawable.obstacle_east;
                case Obstacle.SOUTH:
                    return R.drawable.obstacle_south;
                case Obstacle.WEST:
                    return R.drawable.obstacle_west;
                default:
                    return R.drawable.cell_border;
            }
        }else{
            switch (direction) {
                case Robot.EAST:
                    return R.drawable.car_east;
                case Robot.SOUTH:
                    return R.drawable.car_south;
                case Robot.WEST:
                    return R.drawable.car_west;
                default:
                    return R.drawable.car_north;
            }
        }

    }
    private void rotateObstacle(Obstacle obstacle) {
        obstacle.rotate();
        imageViews[obstacle.getRow()][obstacle.getCol()].setImageResource(getDrawableForDirection("obstacle",obstacle.getDirection()));
    }
    private void placeRobot(int row, int col, int direction) {
        Log.d("placeRobot","row: " +row);
        Log.d("placeRobot","col: " +col);
        if (row - 1 < 0 || row + 1 >= gridSize || col - 1 < 0 || col + 1 >= gridSize) {
            Toast.makeText(getContext(), "The robot cannot be placed here.", Toast.LENGTH_SHORT).show();
            return;
        }
        for (int i = row - 1; i <= row + 1; i++) {
            for (int j = col - 1; j <= col + 1; j++) {
                Obstacle obstacle = getObstacleAt(i, j);
                if (obstacle != null) {
                    Toast.makeText(getContext(), "The robot cannot be placed on an obstacle.", Toast.LENGTH_SHORT).show();
                    return;
                }
            }
        }
        ImageView robotView = new ImageView(getContext());
        myRobot = new Robot(row, col, direction);
        robotView.setImageResource(getDrawableForDirection("robot", myRobot.getDirection()));
        robotView.setBackgroundResource(R.drawable.cell_border);
        GridLayout.LayoutParams robotLayoutParams = new GridLayout.LayoutParams();
        robotLayoutParams.width = 0;
        robotLayoutParams.height = 0;
        robotLayoutParams.rowSpec = GridLayout.spec(gridSize - 2 - col, 3, 1f);
        robotLayoutParams.columnSpec = GridLayout.spec(row-1, 3, 1f);
        robotView.setLayoutParams(robotLayoutParams);
        if (robotImageView != null) {
            gridLayout.removeView(robotImageView);
        }
        gridLayout.addView(robotView);
        robotImageView = robotView;
        myRobot.setRow(row);
        myRobot.setCol(col);
        Log.d("placeRobot2","placeRobot: "+ myRobot.toString());
    }
    private void rotateRobot() {
        myRobot.rotate();
        robotImageView.setImageResource(getDrawableForDirection("robot", myRobot.getDirection()));
        if(MainActivity.bluetoothClient != null){
            MainActivity.bluetoothClient.sendData("rotateRobot: " + myRobot.toString());
        }
    }
    private void updateButtonState(Button button, boolean isActive) {
        if (isActive) {
            button.setBackgroundColor(Color.LTGRAY);
        } else {
            button.setBackgroundColor(Color.BLUE);
        }
    }
    private boolean isCellOccupiedByRobot(int row, int col) {
        if (myRobot != null) {
            for (int i = col - 1; i <= col + 1; i++) {
                for (int j = row - 1; j <= row + 1; j++) {
                    if (i == myRobot.getCol() && j == myRobot.getRow()) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    public static void hideKeyboardFrom(Context context, View view) {
        InputMethodManager imm = (InputMethodManager) context.getSystemService(Activity.INPUT_METHOD_SERVICE);
        imm.hideSoftInputFromWindow(view.getWindowToken(), 0);
    }
    private void moveRobotUp() {
        if(myRobot!=null){
            switch(myRobot.getDirection()){
                case 0:
                    if(myRobot.getCol() + 1 < gridSize && !isAreaOccupiedByObstacle(myRobot.getRow(), myRobot.getCol() + 1)) {
                        placeRobot(myRobot.getRow(), myRobot.getCol()+1, myRobot.getDirection());
                        Log.i("moveRobotUp case 0", "Robot: " + myRobot.toString());
                        sendFW();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be move up.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 2:
                    if(myRobot.getRow() + 1 < gridSize && !isAreaOccupiedByObstacle(myRobot.getRow()+1, myRobot.getCol())) {
                        placeRobot(myRobot.getRow()+1, myRobot.getCol(), myRobot.getDirection());
                        Log.i("moveRobotUp case 2", "Robot: " + myRobot.toString());
                        sendFW();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be move up.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 4:
                    if(myRobot.getCol() - 1 >= 0 && !isAreaOccupiedByObstacle(myRobot.getRow(), myRobot.getCol()-1)) {
                        placeRobot(myRobot.getRow(), myRobot.getCol()-1, myRobot.getDirection());
                        Log.i("moveRobotUp case 4", "Robot: " + myRobot.toString());
                        sendFW();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be move up.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 6:
                    if(myRobot.getRow() - 1 >= 0 && !isAreaOccupiedByObstacle(myRobot.getRow()-1, myRobot.getCol())) {
                        placeRobot(myRobot.getRow()-1, myRobot.getCol(), myRobot.getDirection());
                        Log.i("moveRobotUp case 6", "Robot: " + myRobot.toString());
                        sendFW();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be move up.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                default:
                    Toast.makeText(getContext(), "The robot cannot be move up.", Toast.LENGTH_SHORT).show();
            }
        }else {
            Toast.makeText(getContext(), "Robot Not Found!", Toast.LENGTH_SHORT).show();
        }
    }

    private void moveRobotDown() {

        if(myRobot!=null){
            switch(myRobot.getDirection()){
                case 0:
                    if(myRobot.getCol() - 1 >= 0 && !isAreaOccupiedByObstacle(myRobot.getRow(), myRobot.getCol()-1)) {
                        placeRobot(myRobot.getRow(), myRobot.getCol()-1, myRobot.getDirection());
                        Log.i("moveRobotDown case 0", "Robot: " + myRobot.toString());
                        sendBW();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be move down.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 2:
                    if(myRobot.getRow() - 1 >= 0 && !isAreaOccupiedByObstacle(myRobot.getRow()-1, myRobot.getCol())) {
                        placeRobot(myRobot.getRow()-1, myRobot.getCol(), myRobot.getDirection());
                        Log.i("moveRobotDown case 2", "Robot: " + myRobot.toString());
                        sendBW();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be move down.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 4:
                    if(myRobot.getCol() + 1 < gridSize  && !isAreaOccupiedByObstacle(myRobot.getRow(), myRobot.getCol()+1)) {
                        placeRobot(myRobot.getRow(), myRobot.getCol()+1, myRobot.getDirection());
                        Log.i("moveRobotDown case 4", "Robot: " + myRobot.toString());
                        sendBW();
                    }
                    break;
                case 6:
                    if(myRobot.getRow() + 1 < gridSize && !isAreaOccupiedByObstacle(myRobot.getRow()+1, myRobot.getCol())) {
                        placeRobot(myRobot.getRow()+1, myRobot.getCol(), myRobot.getDirection());
                        Log.i("moveRobotDown case 6", "Robot: " + myRobot.toString());
                        sendBW();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be move down.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                default:
                    Toast.makeText(getContext(), "The robot cannot be move down.", Toast.LENGTH_SHORT).show();
            }
        }else {
            Toast.makeText(getContext(), "Robot Not Found!", Toast.LENGTH_SHORT).show();
        }
    }
/*
    private void moveRobotLeft() {
        if (myRobot.getCol() - 1 >= 0 && !isAreaOccupiedByObstacle(myRobot.getRow(), myRobot.getCol() - 1)) {
            placeRobot(myRobot.getRow(), myRobot.getCol() - 1, myRobot.getDirection());
        } else {
            Toast.makeText(getContext(), "The robot cannot be moved left.", Toast.LENGTH_SHORT).show();
        }
        if(myRobot != null){
            Log.i("moveRobotLeft", "Robot: " + myRobot.toString());
        }
    }
    private void moveRobotRight() {
        if (myRobot.getCol() + 2 < gridSize && !isAreaOccupiedByObstacle(myRobot.getRow(), myRobot.getCol() + 1)) {
            placeRobot(myRobot.getRow(), myRobot.getCol() + 1, myRobot.getDirection());
        } else {
            Toast.makeText(getContext(), "The robot cannot be moved right.", Toast.LENGTH_SHORT).show();
        }
        if(myRobot != null){
            Log.i("moveRobotRight", "Robot: " + myRobot.toString());
        }
    }
 */

    private void moveRobotUpRight() {

        if(myRobot!=null){
            switch(myRobot.getDirection()){
                case 0:
                    if(myRobot.getRow() +3 < gridSize  && myRobot.getCol() +1 < gridSize
                            && !isAreaOccupiedByObstacle(myRobot.getRow()+3, myRobot.getCol()+1)){
                        placeRobot(myRobot.getRow()+3, myRobot.getCol()+1, myRobot.getDirection()+2);
                        Log.i("moveRobotUpRight case 0", "Robot: " + myRobot.toString());
                        sendFR();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved up right.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 2:
                    if(myRobot.getRow() +1 < gridSize  && myRobot.getCol() - 3 >=0
                            && !isAreaOccupiedByObstacle(myRobot.getRow()+1, myRobot.getCol()-3)){
                        placeRobot(myRobot.getRow()+1, myRobot.getCol()-3, myRobot.getDirection()+2);
                        Log.i("moveRobotUpRight case 2", "Robot: " + myRobot.toString());
                        sendFR();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved up right.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 4:
                    if(myRobot.getRow() -3 >=0  && myRobot.getCol() -1 >=0
                            && !isAreaOccupiedByObstacle(myRobot.getRow()-3, myRobot.getCol()-1)){
                        placeRobot(myRobot.getRow()-3, myRobot.getCol()-1, myRobot.getDirection()+2);
                        Log.i("moveRobotUpRight case 4", "Robot: " + myRobot.toString());
                        sendFR();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved up right.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 6:
                    if(myRobot.getRow() - 1 >=0 && myRobot.getCol() +3 < gridSize
                            && !isAreaOccupiedByObstacle(myRobot.getRow()-1, myRobot.getCol()+3)){
                        placeRobot(myRobot.getRow()-1, myRobot.getCol()+3, 0);
                        Log.i("moveRobotUpRight case 6", "Robot: " + myRobot.toString());
                        sendFR();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved up right.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                default:
                    Toast.makeText(getContext(), "The robot cannot be moved up right.", Toast.LENGTH_SHORT).show();
            }
        }else {
            Toast.makeText(getContext(), "Robot Not Found!", Toast.LENGTH_SHORT).show();
        }
    }

    private void moveRobotUpLeft() {
        if(myRobot!=null){
            switch(myRobot.getDirection()){
                case 0:
                    if(myRobot.getRow() -3 >=0  && myRobot.getCol() +1 < gridSize
                            && !isAreaOccupiedByObstacle(myRobot.getRow()-3, myRobot.getCol()+1)){
                        placeRobot(myRobot.getRow()-3, myRobot.getCol()+1, 6);
                        Log.i("moveRobotUpRight case 0", "Robot: " + myRobot.toString());
                        sendFL();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved up left.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 2:
                    if(myRobot.getRow() +1 < gridSize  && myRobot.getCol() +3 < gridSize
                            && !isAreaOccupiedByObstacle(myRobot.getRow()+1, myRobot.getCol()+3)){
                        placeRobot(myRobot.getRow()+1, myRobot.getCol()+3, myRobot.getDirection()-2);
                        Log.i("moveRobotUpRight case 2", "Robot: " + myRobot.toString());
                        sendFL();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved up left.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 4:
                    if(myRobot.getRow() +3 < gridSize  && myRobot.getCol() -1 >=0
                            && !isAreaOccupiedByObstacle(myRobot.getRow()+3, myRobot.getCol()-1)){
                        placeRobot(myRobot.getRow()+3, myRobot.getCol()-1, myRobot.getDirection()-2);
                        Log.i("moveRobotUpRight case 4", "Robot: " + myRobot.toString());
                        sendFL();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved up left.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 6:
                    if(myRobot.getRow() - 1 >=0 && myRobot.getCol() -3 >=0
                            && !isAreaOccupiedByObstacle(myRobot.getRow()-1, myRobot.getCol()-3)){
                        placeRobot(myRobot.getRow()-1, myRobot.getCol()-3, myRobot.getDirection()-2);
                        Log.i("moveRobotUpRight case 6", "Robot: " + myRobot.toString());
                        sendFL();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved up left.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                default:
                    Toast.makeText(getContext(), "The robot cannot be moved up left.", Toast.LENGTH_SHORT).show();
            }
        }else {
            Toast.makeText(getContext(), "Robot Not Found!", Toast.LENGTH_SHORT).show();
        }
    }

    private void moveRobotBottomRight() {
        if(myRobot!=null){
            switch(myRobot.getDirection()){
                case 0:
                    if(myRobot.getRow() +1 < gridSize  && myRobot.getCol() -3 >=0
                            && !isAreaOccupiedByObstacle(myRobot.getRow()+1, myRobot.getCol()-3)){
                        placeRobot(myRobot.getRow()+1, myRobot.getCol()-3, 6);
                        Log.i("moveRobotUpRight case 0", "Robot: " + myRobot.toString());
                        sendBR();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved bottom right.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 2:
                    if(myRobot.getRow() -3 >=0   && myRobot.getCol() -1 >=0
                            && !isAreaOccupiedByObstacle(myRobot.getRow()-3, myRobot.getCol()-1)){
                        placeRobot(myRobot.getRow()-3, myRobot.getCol()-1, myRobot.getDirection()-2);
                        Log.i("moveRobotUpRight case 2", "Robot: " + myRobot.toString());
                        sendBR();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved bottom right.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 4:
                    if(myRobot.getRow() -1 >=0  && myRobot.getCol() +3 < gridSize
                            && !isAreaOccupiedByObstacle(myRobot.getRow()-1, myRobot.getCol()+3)){
                        placeRobot(myRobot.getRow()-1, myRobot.getCol()+3, myRobot.getDirection()-2);
                        Log.i("moveRobotUpRight case 4", "Robot: " + myRobot.toString());
                        sendBR();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved bottom right.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 6:
                    if(myRobot.getRow() + 3 < gridSize && myRobot.getCol() +1 < gridSize
                            && !isAreaOccupiedByObstacle(myRobot.getRow()+3, myRobot.getCol()+1)){
                        placeRobot(myRobot.getRow()+3, myRobot.getCol()+1, myRobot.getDirection()-2);
                        Log.i("moveRobotUpRight case 6", "Robot: " + myRobot.toString());
                        sendBR();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved bottom right.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                default:
                    Toast.makeText(getContext(), "The robot cannot be moved right.", Toast.LENGTH_SHORT).show();
            }
        }else {
            Toast.makeText(getContext(), "Robot Not Found!", Toast.LENGTH_SHORT).show();
        }
    }

    private void moveRobotBottomLeft() {
        if(myRobot!=null){
            switch(myRobot.getDirection()){
                case 0:
                    if(myRobot.getRow() -1 >=0  && myRobot.getCol() -3 >=0
                            && !isAreaOccupiedByObstacle(myRobot.getRow()-1, myRobot.getCol()-3)){
                        placeRobot(myRobot.getRow()-1, myRobot.getCol()-3, myRobot.getDirection()+2);
                        Log.i("moveRobotUpRight case 0", "Robot: " + myRobot.toString());
                        sendBL();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved bottom left.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 2:
                    if(myRobot.getRow() -3 >=0  && myRobot.getCol() +1 < gridSize
                            && !isAreaOccupiedByObstacle(myRobot.getRow()-3, myRobot.getCol()+1)){
                        placeRobot(myRobot.getRow()-3, myRobot.getCol()+1, myRobot.getDirection()+2);
                        Log.i("moveRobotUpRight case 2", "Robot: " + myRobot.toString());
                        sendBL();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved bottom left.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 4:
                    if(myRobot.getRow() +1 < gridSize  && myRobot.getCol() +3 < gridSize
                            && !isAreaOccupiedByObstacle(myRobot.getRow()+1, myRobot.getCol()+3)){
                        placeRobot(myRobot.getRow()+1, myRobot.getCol()+3, myRobot.getDirection()+2);
                        Log.i("moveRobotUpRight case 4", "Robot: " + myRobot.toString());
                        sendBL();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved bottom left.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 6:
                    if(myRobot.getRow() +3 < gridSize && myRobot.getCol() -1 < gridSize
                            && !isAreaOccupiedByObstacle(myRobot.getRow()+3, myRobot.getCol()-1)){
                        placeRobot(myRobot.getRow()+3, myRobot.getCol()-1, 0);
                        Log.i("moveRobotUpRight case 6", "Robot: " + myRobot.toString());
                        sendBL();
                    }else{
                        Toast.makeText(getContext(), "The robot cannot be moved bottom left.", Toast.LENGTH_SHORT).show();
                    }
                    break;
                default:
                    Toast.makeText(getContext(), "The robot cannot be moved bottom left.", Toast.LENGTH_SHORT).show();
            }
        }else {
            Toast.makeText(getContext(), "Robot Not Found!", Toast.LENGTH_SHORT).show();
        }
    }
    private boolean validateSetUpCoordinate(EditText val){
        String editTextValueString = val.getText().toString();
        Log.d("validateSetUpCoordinate", "editTextValueString >>> "+editTextValueString);
        if (!editTextValueString.isEmpty()) {
            int editTextValueInt = Integer.parseInt(editTextValueString);
            if (editTextValueInt < 0 || editTextValueInt > 19) {
                return false;
            }
        }
        return true;
    }

    private boolean validateSetUpDirection(EditText val){
        String editTextValueString = val.getText().toString();
        Log.d("validateSetUpDirection", "editTextValueString >>> "+editTextValueString);
        if (!editTextValueString.isEmpty()) {
            int editTextValueInt = Integer.parseInt(editTextValueString);
            if (editTextValueInt == 0 || editTextValueInt == 2 || editTextValueInt == 4 || editTextValueInt == 6) {
                return true;
            }
        }
        return false;
    }
    private boolean isAreaOccupiedByObstacle(int row, int col) {
        // Check each cell in the 3x3 grid of the target area
        for (int i = row - 1; i <= row + 1; i++) {
            for (int j = col - 1; j <= col + 1; j++) {
                // If the cell is occupied by an obstacle, return true
                if (getObstacleAt(i, j) != null) {
                    return true;
                }
            }
        }
        return false;
    }
/*
    private String getTextBeforeColon(String input) {
        int colonIndex = input.indexOf(':');
        if (colonIndex != -1) {
            return input.substring(0, colonIndex).trim().toLowerCase();
        }
        return input;
    }
*/
    private void sendFL(){
        if(MainActivity.bluetoothClient != null) {
            MainActivity.bluetoothClient.sendData("FL");
        }
    }

    private void sendFW(){
        if(MainActivity.bluetoothClient != null){
            MainActivity.bluetoothClient.sendData("FW");
        }
    }
    private void sendFR(){
        if(MainActivity.bluetoothClient != null) {
            MainActivity.bluetoothClient.sendData("FR");
        }
    }
    private void sendBL(){
        if(MainActivity.bluetoothClient != null) {
            MainActivity.bluetoothClient.sendData("BL");
        }
    }
    private void sendBW(){
        if(MainActivity.bluetoothClient != null) {
            MainActivity.bluetoothClient.sendData("BW");
        }
    }
    private void sendBR() {
        if (MainActivity.bluetoothClient != null) {
            MainActivity.bluetoothClient.sendData("BR");
        }
    }
    public String createSetUpJson(List<Obstacle> obstacles, Robot robot) {
        JSONObject jsonObject = new JSONObject();
        JSONArray jsonArray = new JSONArray();
        for (Obstacle obstacle : obstacles) {
            JSONObject jsonObstacle = new JSONObject();
            try {
                jsonObstacle.put("x", obstacle.getRow());
                jsonObstacle.put("y", obstacle.getCol());
                jsonObstacle.put("d", obstacle.getDirection());
                jsonObstacle.put("id", obstacle.getId());
                jsonArray.put(jsonObstacle);
            } catch (JSONException e) {
                e.printStackTrace();
            }
        }
        try {
            jsonObject.put("obstacles", jsonArray);
            jsonObject.put("robot_x", robot.getRow());
            jsonObject.put("robot_y", robot.getCol());
            jsonObject.put("robot_dir", robot.getDirection());
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return jsonObject.toString();
    }

    public void updateReceivedData(String data){
        try {
            String[] splitString = null;
            String[] splitStringPipe = null;
            String[] splitStringPipeData = null;

            if (data.startsWith("ROBOT")) {
                splitString = data.split(",");
                for (String receivedData : splitString) {
                    Log.d(TAG, "splitString: " + receivedData.trim());
                }
                if (splitString != null) {
                    int direction = Integer.parseInt(splitString[3].trim());
                    int xCoor = Integer.parseInt(splitString[1].trim());
                    int yCoor = Integer.parseInt(splitString[2].trim());
                    int snapStatus = Integer.parseInt(splitString[4].trim());
                    Log.d(TAG, "direction: " + direction);
                    Log.d(TAG, "xCoor: " + xCoor);
                    Log.d(TAG, "yCoor: " + yCoor);
                    //Log.d(TAG,"snapStatus: " +snapStatus);
                    if (snapStatus != -1) {
                        statusTextArea.setText("Snapping");
                    } else {
                        statusTextArea.setText("Moving");
                    }
                    if (xCoor != 0 && yCoor != 0 && direction != 0) {
                        placeRobot(xCoor, yCoor, direction);
                    }
                    receivedDataTextArea.setText(xCoor + "," + yCoor + "," + direction);
                }
            }
            else if (data.startsWith("TARGET")) {
                splitString = data.split(",");
                for (String receivedData : splitString) {
                    Log.d(TAG, "splitString: " + receivedData.trim());
                }
                List<Integer> obstacleIDArr = new ArrayList<>();
                List<Integer> imgIDArr = new ArrayList<>();
                if (splitString != null) {
                    for (int i = 1; i < splitString.length; i++) {
                        if (i % 2 == 1) {
                            //even -> obs id
                            obstacleIDArr.add(Integer.parseInt(splitString[i]));
                        } else {
                            //odd -> obs img id
                            imgIDArr.add(Integer.parseInt(splitString[i]));
                        }
                    }


                        //loop obs
                    for (int i = 0; i < obstacleIDArr.size(); i++) {
                        for (Obstacle obstacle : obstacles) {
                            if (obstacle.getId() == obstacleIDArr.get(i)) {
                                obstacle.setImageId(imgIDArr.get(i));

                                FrameLayout frameLayout = frameLayouts[obstacle.getRow()][obstacle.getCol()];
                                imageViews[obstacle.getRow()][obstacle.getCol()].setImageResource(getDrawableForDirection("obstacle", obstacle.getDirection()));
                                Log.i(TAG, "Obstacle Obj: " + obstacle.toString());
                                TextView textView = (TextView) frameLayout.getChildAt(1);  // get the TextView
                                textView.setText(String.valueOf(obstacle.getImageId()));
                                textView.setTextColor(Color.GREEN);
                                textView.setVisibility(View.VISIBLE);  // make it visible
                            }
                        }
                    }
                }
            }
        }catch(ArrayIndexOutOfBoundsException e){
            //Toast.makeText(getContext(), "Unable to update received data...", Toast.LENGTH_SHORT).show();
            Log.d(TAG, "updateReceivedData:" + e);
        }
    }
    /*public void updateReceivedData(String data){
        try {
            String[] splitString = null;
            String[] splitStringPipe = null;
            String[] splitStringPipeData = null;

            splitString = data.split(",");
            splitStringPipe = data.split("\\|");
            //for loop for testing purpose...
            for (String receivedData : splitString) {
                Log.d(TAG,"splitString: " +receivedData.trim());
            }

            if (splitString != null) {
                int direction = Integer.parseInt(splitString[0].trim());
                int xCoor = Integer.parseInt(splitString[1].trim());
                int yCoor = Integer.parseInt(splitString[2].trim());
                int snapStatus = Integer.parseInt(splitString[3].trim());
                Log.d(TAG,"direction: " +direction);
                Log.d(TAG,"xCoor: " +xCoor);
                Log.d(TAG,"yCoor: " +yCoor);
                Log.d(TAG,"snapStatus: " +snapStatus);
                if(snapStatus != -1){
                    statusTextArea.setText("Snapping");
                }else{
                    statusTextArea.setText("Moving");
                }
                if(xCoor!=0 && yCoor!=0 && direction!=0){
                    placeRobot(xCoor,yCoor,direction);
                }


                receivedDataTextArea.setText(xCoor+","+yCoor+","+direction);
            }
            if(splitStringPipe != null){
                String stringAfterPipe = splitStringPipe[1];
                splitStringPipeData = stringAfterPipe.split(",");
                List<Integer> obstacleIDArr =  new ArrayList<>();
                List<Integer>imgIDArr = new ArrayList<>();
                if(splitStringPipeData!=null){
                    for(int i=0;i<splitStringPipeData.length;i++){
                        if(i%2==0){
                            //even -> obs id
                            obstacleIDArr.add(Integer.parseInt(splitStringPipeData[i]));
                        }else{
                            //odd -> obs img id
                            imgIDArr.add(Integer.parseInt(splitStringPipeData[i]));
                        }
                    }

                    //loop obs
                    for(int i=0; i<obstacleIDArr.size();i++){
                        for (Obstacle obstacle : obstacles) {
                            if (obstacle.getId() == obstacleIDArr.get(i)) {
                                obstacle.setImageId(imgIDArr.get(i));

                                FrameLayout frameLayout = frameLayouts[obstacle.getRow()][obstacle.getCol()];
                                imageViews[obstacle.getRow()][obstacle.getCol()].setImageResource(getDrawableForDirection("obstacle", obstacle.getDirection()));
                                Log.i(TAG, "Obstacle Obj: " + obstacle.toString());
                                TextView textView = (TextView) frameLayout.getChildAt(1);  // get the TextView
                                textView.setText(String.valueOf(obstacle.getImageId()));
                                textView.setTextColor(Color.GREEN);
                                textView.setVisibility(View.VISIBLE);  // make it visible
                            }
                        }
                    }
                }
            }
        }catch (ArrayIndexOutOfBoundsException e){
            //Toast.makeText(getContext(), "Unable to update received data...", Toast.LENGTH_SHORT).show();
            Log.d(TAG,"updateReceivedData:" +e);
        }
    }*/
}