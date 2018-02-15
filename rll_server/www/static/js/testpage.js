// A little Javascript that showcases the Robot Learning Lab's REST API

$(function() {

    var form = $("#submit-form");
    var subMessages = $("#submission-messages");
    var jobMessages = $("#status-messages");
    var jobIDdiv = $("#job-id-div");
    var logsDownload = $("#logs-download");


    // Set up an event listener for the submit form
    $(form).submit(function(e) {
        // Stop the browser from submitting the form
        e.preventDefault();

        // Serialize the form data
        var formData = $(form).serialize();

        // Submit the form
        $.ajax({
            type: "POST",
            url: $(form).attr("action"),
            data: formData
        })
            .done(function(response) {
                var obj = JSON.parse(response);

                // Clear the form and status divs
                $("#username").val("");
                $("#repo_url").val("");
                $(jobIDdiv).removeClass("success");
                $(jobIDdiv).removeClass("error");
                $(jobIDdiv).text("")
                $(jobMessages).removeClass("success");
                $(jobMessages).removeClass("error");
                $(jobMessages).text("");
                $(logsDownload).removeClass("button");

                if (obj.status == "success") {
                    $(subMessages).removeClass("error");
                    $(subMessages).addClass("success");
                    $(subMessages).text("job successfully submitted");
                    poll_status(obj.job_id);
                } else if (obj.status == "error") {
                    $(subMessages).removeClass("success");
                    $(subMessages).addClass("error");
                    $(subMessages).text(obj.error);
                }
            })
            .fail(function(data) {
                $(subMessages).removeClass("success");
                $(subMessages).addClass("error");

                if (data.responseText !== "") {
                    $(subMessages).text(data.responseText);
                } else {
                    $(subMessages).text("An error occured.");
                }
            });
    });

    // poll job status
    function poll_status(job_id) {
        var job_done_or_error = false;

        $(jobIDdiv).addClass("success");
        $(jobIDdiv).text("Job ID: " + job_id);

        (function poll(job_id, job_done_or_error) {
            setTimeout(function() {
                var job_status;
                $.getJSON("http://localhost:8888/jobs?op=status&job=" + job_id, function(obj) {
                    if (obj.status == "success") {
                        $(jobMessages).addClass("success");
                        job_status = "Status: " + obj.job_status;
                    } else if (obj.status == "error") {
                        $(jobMessages).addClass("error");
                        job_status = obj.error;
                        job_done_or_error = true;
                    }

                    if (obj.job_status == "finished") {
                        job_status += "; Job Result: " + obj.job_result;
                        advertise_logs(job_id);
                        job_done_or_error = true;
                    }

                    $(jobMessages).text(job_status);

                    if (!job_done_or_error)
                        poll(job_id, job_done_or_error);
                });
            }, 1000);
        })(job_id, job_done_or_error);
    };

    function advertise_logs(job_id) {
        $.getJSON("http://localhost:8888/jobs?op=logs&job=" + job_id, function(obj) {
            $(logsDownload).addClass("log_link");
            // var dl_button = document.createElement("button");
            var link = document.createElement("a");
            // link.addClass("log_link");
            link.innerHTML = "Download Log";
            link.href = obj.log_url;
            link.download = "job.log";

            // link.addEventListener("click", function(ev) {
            //     link.href = obj.log_url;
            //     link.download = "job.log";
            // }, false);
            var download_div = document.getElementById("logs-download");
            // dl_button.appendChild(link);
            download_div.appendChild(link);
            // $(logsDownload).text("Logs URL: " + obj.log_url);
        });
    };
});
