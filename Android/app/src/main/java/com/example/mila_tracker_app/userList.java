package com.example.mila_tracker_app;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.drawable.BitmapDrawable;
import android.os.Bundle;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import java.util.ArrayList;



public class userList extends AppCompatActivity {


//    RecyclerView recyclerView;
//    DatabaseReference database;
//    ListAdapter listAdapter;
//    ArrayList<User> list;

//    @Override
//    protected void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_user_list);
//
//
//        recyclerView = findViewById(R.id.userList);
//        database = FirebaseDatabase.getInstance().getReference("mlx90614/2-push");
//        System.out.println("@@@@");
//        System.out.println(database);
//        recyclerView.setHasFixedSize(true);
//        recyclerView.setLayoutManager(new LinearLayoutManager(this));
//
//        list = new ArrayList<>();
//        listAdapter = new ListAdapter(this, list);
//        recyclerView.setAdapter(listAdapter);
//
//        database.addValueEventListener(new ValueEventListener() {
//            @Override
//            public void onDataChange(@NonNull DataSnapshot snapshot) {
//
//                for (DataSnapshot dataSnapshot : snapshot.getChildren()) {
//
//                    User user = dataSnapshot.getValue(User.class);
//                    list.add(user);
//
//
//                }
//                listAdapter.notifyDataSetChanged();
//
//            }
//
//            @Override
//            public void onCancelled(@NonNull DatabaseError error) {
//
//            }
//        });
//
//    }
    TextView txtData;
    DatabaseReference database;
    ListAdapter listAdapter;
    ArrayList<User> list;
    private Bitmap bmp;
    private Bitmap operation;
    ImageView im;
    double[][] anchor = {{0,3400,3400,0},{0,0,3400,3400}};
    double[][] anchor_img = {{10,390,390,10},{10,10,390,390}};//{{10,790,790,10},{10,10,790,790}};
    int dot_radius = 10;
    int imgW=400,imgH=400;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.data_print);

        // For Displaying Image
        im = (ImageView) findViewById(R.id.imgMap);
        BitmapDrawable abmp = (BitmapDrawable) im.getDrawable();
        bmp = abmp.getBitmap();

        operation = Bitmap.createBitmap(imgW,imgH, bmp.getConfig());//Bitmap.createBitmap(bmp.getWidth(),bmp.getHeight(), bmp.getConfig());
        for (int i = 0; i < operation.getWidth(); i++) {
            for (int j = 0; j < operation.getHeight(); j++) {

                if( Math.pow(i-anchor_img[0][0],2) + Math.pow(j-anchor_img[1][0],2) <=Math.pow(dot_radius,2)
                || Math.pow(i-anchor_img[0][1],2) + Math.pow(j-anchor_img[1][1],2) <=Math.pow(dot_radius,2)
                || Math.pow(i-anchor_img[0][2],2) + Math.pow(j-anchor_img[1][2],2) <=Math.pow(dot_radius,2)
                || Math.pow(i-anchor_img[0][3],2) + Math.pow(j-anchor_img[1][3],2) <=Math.pow(dot_radius,2))
                {
                    int r = 0;
                    int g = 0;
                    int b = 0;
                    operation.setPixel(i, j, Color.rgb( r, g, b));
                }else {

                    int r = 255;
                    int g = 255;
                    int b = 255;
                    operation.setPixel(i, j, Color.rgb(r, g, b));
                }
            }
        }
        im.setImageBitmap(operation);
        abmp = (BitmapDrawable) im.getDrawable();
        bmp = abmp.getBitmap();
        System.out.println(bmp.getWidth());

        txtData = findViewById(R.id.data_display);
        database = FirebaseDatabase.getInstance().getReference("mlx90614/5-push");
        System.out.println("@@@@");
        System.out.println(database);
        System.out.println(database.push().getParent());
//        recyclerView.setHasFixedSize(true);
//        recyclerView.setLayoutManager(new LinearLayoutManager(this));

        list = new ArrayList<>();
        listAdapter = new ListAdapter(this, list);
//        recyclerView.setAdapter(listAdapter);

        database.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot snapshot) {
                String all_str = "C: ";
                for (DataSnapshot dataSnapshot : snapshot.getChildren()) {

//                    double x = dataSnapshot.getValue(double.class);
//                    all_str = all_str + "  "+ x;
                    User user = dataSnapshot.getValue(User.class);
                    String str = "Coordinate: "+ user.getAmbient() + ", " + user.getObject()+ '\n';
                    all_str = str;

                    double loc_x_mm = user.getAmbient();
                    double loc_y_mm = user.getObject();

                    int map_x =  (int) ((loc_x_mm-anchor[0][0])/5000*imgW+anchor_img[0][0]);
                    int map_y =  (int) ((loc_y_mm-anchor[1][0])/5000*imgH+anchor_img[1][0]);


                    operation = Bitmap.createBitmap(imgW,imgH, bmp.getConfig());//Bitmap.createBitmap(bmp.getWidth(),bmp.getHeight(), bmp.getConfig());
                    for (int i = 0; i < operation.getWidth(); i++) {
                        for (int j = 0; j < operation.getHeight(); j++) {
                            int p = bmp.getPixel(i, j);

                            if( Math.pow(i-map_x,2) + Math.pow(j-map_y,2) <=Math.pow(dot_radius,2))
                            {
                                int r = 0;
                                int g = 0;
                                int b = 128;
                                operation.setPixel(i, j, Color.rgb( r, g, b));
                            }else {
                                operation.setPixel(i, j, Color.rgb(Color.red(p), Color.green(p), Color.blue(p)));
                            }
                        }
                    }
                    im.setImageBitmap(operation);
                }
//                snapshot.getChildren().
//                listAdapter.notifyDataSetChanged();
                txtData.setText(all_str);




            }

            @Override
            public void onCancelled(@NonNull DatabaseError error) {

            }
        });

    }
}
