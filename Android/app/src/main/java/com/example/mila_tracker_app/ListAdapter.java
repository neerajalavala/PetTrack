package com.example.mila_tracker_app;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import org.w3c.dom.Text;

import java.util.ArrayList;

public class ListAdapter extends RecyclerView.Adapter<ListAdapter.MyViewHolder> {

    Context context;

    ArrayList<User> list;


    public ListAdapter(Context context, ArrayList<User> list) {
        this.context = context;
        this.list = list;
    }

    @NonNull
    @Override
    public MyViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        View v = LayoutInflater.from(context).inflate(R.layout.item,parent,false);
        return  new MyViewHolder(v);
    }

    @Override
    public void onBindViewHolder(@NonNull MyViewHolder holder, int position) {

        User user = list.get(position);
        holder.ambient.setText(""+user.getAmbient());
        holder.object.setText(""+user.getObject());


    }

    @Override
    public int getItemCount() {
        return list.size();
    }

    public static class MyViewHolder extends RecyclerView.ViewHolder{

        TextView ambient, object;

        public MyViewHolder(@NonNull View itemView) {
            super(itemView);

            ambient = itemView.findViewById(R.id.tvambient);
            object = itemView.findViewById(R.id.tvobject);

        }
    }


}
