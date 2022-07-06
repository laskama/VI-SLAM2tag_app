/*
 * Copyright 2022 Marius Laska
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.laskama.vislam2tag;

import android.net.wifi.ScanResult;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.Callable;

public class WriteWLAN implements Callable<Integer> {

    private final List<ScanResult> scanResults;
    private final FileWriter writer;
    private static int counter = 0;

    public WriteWLAN(List<ScanResult> scanResults, FileWriter writer) {
        this.scanResults = scanResults;
        this.writer = writer;
    }

    @Override
    public Integer call() {
        // Some long running task
        try {
            for (ScanResult scanResult: scanResults) {
                String line = String.format(
                        "%d; %d; WLAN; %s; %s; %d\n",
                        counter,
                        scanResult.timestamp,
                        scanResult.SSID,
                        scanResult.BSSID,
                        scanResult.level);

                System.out.println("WROTE WIFI FILE");
                writer.write(line);
            }
            writer.flush();
            counter ++;
        } catch (IOException e) {
            e.printStackTrace();
        }

        return 0;
    }
}
