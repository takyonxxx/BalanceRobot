package com.balancerobot;

import android.content.Context;
import android.view.Gravity;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup.LayoutParams;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class PIDSet {
	
	public static void showPIDSetDialog(Context context, View parent) {
		final String TAG = "BalanceRobotControl";
		final TextView textView_KP;
		final TextView textView_KI;
		final TextView textView_KD;

		final SeekBar seekBar_KP;
		final SeekBar seekBar_KI;
		final SeekBar seekBar_KD;

		View paramView = View.inflate(context, R.layout.param_dlg, null);
		LinearLayout contentView = (LinearLayout) paramView
				.findViewById(R.id.param_dlg_content_view);
		View algoLineParamView = View
				.inflate(context, R.layout.pidset, null);
		//final EditText algoLineEdit = (EditText) algoLineParamView
				//.findViewById(R.id.algo_line_edit);
		//algoLineEdit.setText(component.getKAngle() + "");
		contentView.addView(algoLineParamView);
		Button okBtn = (Button) paramView.findViewById(R.id.param_dlg_ok);
		Button canelBtn = (Button) paramView.findViewById(R.id.param_dlg_canel);
		final PopupWindow pinParamDialog = new PopupWindow(paramView,
				LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT, true);
		
		textView_KP = (TextView) paramView.findViewById(R.id.textView_KP);
		textView_KI = (TextView) paramView.findViewById(R.id.textView_KI);
		textView_KD = (TextView) paramView.findViewById(R.id.textView_KD);

		seekBar_KP = (SeekBar) paramView.findViewById(R.id.seekBar_KP);
		seekBar_KP.setMax(1000);
		seekBar_KP.setProgress(MainActivity.getKP());

		seekBar_KI = (SeekBar) paramView.findViewById(R.id.seekBar_KI);
		seekBar_KI.setMax(500);
		seekBar_KI.setProgress(MainActivity.getKI());

		seekBar_KD = (SeekBar) paramView.findViewById(R.id.seekBar_KD);
		seekBar_KD.setMax(100);
		seekBar_KD.setProgress(MainActivity.getKD());


		float KP =(float) seekBar_KP.getProgress();
		String sKP = Float.toString(KP);
		textView_KP.setText("KP = " + sKP);

		float KI =(float) seekBar_KI.getProgress();
		String sKI = Float.toString(KI);
		textView_KI.setText("KI = " + sKI);

		float KD =(float) seekBar_KD.getProgress()  / 5;
		String sKD = Float.toString(KD);
		textView_KD.setText("KD = " + sKD);

		OnSeekBarChangeListener seekBar_KP_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
                                          boolean fromUser) {
				//App.setKAngle(seekBar_KAngle.getProgress());
				MainActivity.setKP(seekBar_KP.getProgress());
				float KP =(float) seekBar_KP.getProgress();
				String sKP = Float.toString(KP);
				textView_KP.setText("KP = " + sKP);
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};

		seekBar_KP.setOnSeekBarChangeListener(seekBar_KP_Listener);


		OnSeekBarChangeListener seekBar_KI_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
                                          boolean fromUser) {
				MainActivity.setKI(seekBar_KI.getProgress());
				float KI =(float) seekBar_KI.getProgress();
				String sKI = Float.toString(KI);
				textView_KI.setText("KI = " + sKI);
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};

		seekBar_KI.setOnSeekBarChangeListener(seekBar_KI_Listener);

		OnSeekBarChangeListener seekBar_KD_Listener = new OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress,
										  boolean fromUser) {
				MainActivity.setKD(seekBar_KD.getProgress());
				float KD =(float) seekBar_KD.getProgress() / 5;
				String sKD = Float.toString(KD);
				textView_KD.setText("KD = " + sKD);
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
				// Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
			}

		};

		seekBar_KD.setOnSeekBarChangeListener(seekBar_KD_Listener);


		okBtn.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View v) {
				pinParamDialog.dismiss();
				//component.setScale(Float.parseFloat(algoLineEdit.getText()
						//.toString()));
				MainActivity.setVisible();
			}
		});
		
		canelBtn.setOnClickListener(new OnClickListener() {
			
			@Override
			public void onClick(View v) {
				pinParamDialog.dismiss();
				MainActivity.setVisible();
			}
		});
		pinParamDialog.showAtLocation(parent, Gravity.CENTER, 0, 0);
	}	
}
