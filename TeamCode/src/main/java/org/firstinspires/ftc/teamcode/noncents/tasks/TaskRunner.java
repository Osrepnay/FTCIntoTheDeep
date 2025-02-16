package org.firstinspires.ftc.teamcode.noncents.tasks;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class TaskRunner {
    List<Task> taskQueue = new LinkedList<>();
    List<Task> runningTasks = new ArrayList<>();
    Set<Object> consumedResources = new HashSet<>();

    public void sendTask(Task task) {
        if (!Collections.disjoint(consumedResources, task.resources)) {
            // remove all conflicting tasks
            boolean removed = false;
            for (int i = 0; i < runningTasks.size(); i++) {
                if (!Collections.disjoint(runningTasks.get(i).resources, task.resources)) {
                    if (runningTasks.get(i).cancellable) {
                        consumedResources.removeAll(runningTasks.get(i).resources);
                        runningTasks.remove(i);
                        removed = true;
                    }
                    break;
                }
            }
            if (!removed) {
                taskQueue.add(task);
            } else {
                if (!task.update.getAsBoolean()) {
                    runningTasks.add(task);
                    consumedResources.addAll(task.resources);
                }
                recheckQueue();
            }
        } else {
            if (!task.update.getAsBoolean()) {
                runningTasks.add(task);
                consumedResources.addAll(task.resources);
            }
        }
    }

    public void update() {
        for (int i = 0; i < runningTasks.size(); i++) {
            if (runningTasks.get(i).update.getAsBoolean()) {
                consumedResources.removeAll(runningTasks.get(i).resources);
                runningTasks.remove(i);
                i--;
                recheckQueue();
            }
        }
    }

    private void recheckQueue() {
        for (int j = 0; j < taskQueue.size(); j++) {
            if (Collections.disjoint(consumedResources, taskQueue.get(j).resources)) {
                runningTasks.add(taskQueue.get(j));
                consumedResources.addAll(taskQueue.get(j).resources);
                taskQueue.remove(j);
                j--;
            }
        }
    }

    public void flush() {
        taskQueue.clear();
        runningTasks.clear();
        consumedResources.clear();
    }
}
