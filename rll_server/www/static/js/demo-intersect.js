// A little Javascript that showcases the Robot Learning Lab's REST API

$(function() {

    var form = $("#submit-form");
    var subMessages = $("#submission-messages");
    var jobMessages = $("#status-messages");
    var jobIDdiv = $("#job-id-div");
    var view_div = document.getElementById("logs-view");
    clear_btn = document.getElementById("clear-button");

    clear_btn.addEventListener('click', function() {
        clear_page();
    }, false);

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
                clear_page();

                if (obj.status == "success") {
                    $(subMessages).removeClass("error");
                    $(subMessages).addClass("success");
                    $(subMessages).text("job successfully submitted");
                    $(form).collapse("hide");
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
                $.getJSON("./api/jobs?op=status&job=" + job_id, function(obj) {
                    if (obj.status == "success") {
                        $(jobMessages).addClass("success");
                        job_status = "Status: " + obj.job_status;
                    } else if (obj.status == "error") {
                        $(jobMessages).addClass("error");
                        job_status = obj.error;
                        job_done_or_error = true;
                    }

                    if (obj.job_status == "running") {
                        stream_cam(true, obj.cam_url);
                    } else if (obj.job_status == "finished") {
                        // end cam stream
                        stream_cam(false);

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

    function stream_cam(run, cam_url) {
        var modal_div = document.getElementById("stream-modal");
        var view_div = document.getElementById("stream-modal-body");

        if (run && document.getElementById("robot_img") != null) {
            // stream is already set up
            return
        } else if (run && cam_url != undefined && cam_url != "unknown") {
            var img = document.createElement("img");
            img.src = cam_url;
            img.id = "robot_img";
            view_div.appendChild(img);
            $(modal_div).modal('show');
        } else {
            $(modal_div).modal('hide');
            var img = document.getElementById("robot_img");
            // makes the browser stop loading the stream
            img.src = "#";
            img.parentNode.removeChild(img);
        }
    };

    function advertise_logs(job_id) {
        $.getJSON("./api/jobs?op=logs&job=" + job_id, function(obj) {
            var download_btn = document.getElementById("log-download-btn");
            download_btn.href = obj.log_url;
            download_btn.download = "job.log";
            download_btn.target = "_blank";

            // clear after log modal is closed
            var close_btn = document.getElementById("log-close");
            close_btn.addEventListener('click', function() {
                clear_page();
            }, false);

            $(view_div).collapse("show");

            log_embed_handle = function() {
                if (document.getElementById("log-embed") == null) {
                    $.get(obj.log_url, function(data) {
                        var pre = document.createElement("pre");
                        pre.id = "log-embed";
                        pre.innerHTML = data;
                        var view_div = document.getElementById("log-modal-body");
                        view_div.appendChild(pre);
                    });
                }
            };

            view_div.addEventListener("click", log_embed_handle, {once: true});
        });
    };

    function clear_page() {
        $(subMessages).removeClass("success");
        $(subMessages).removeClass("error");
        $(subMessages).text("");
        $(jobIDdiv).removeClass("success");
        $(jobIDdiv).removeClass("error");
        $(jobIDdiv).text("")
        $(jobMessages).removeClass("success");
        $(jobMessages).removeClass("error");
        $(jobMessages).text("");
        $(view_div).collapse("hide");
        $(form).collapse("show");
        $("input:radio").prop('checked', false);

        var log = document.getElementById("log-embed");
        if (log != null)
            log.parentNode.removeChild(log);
    };
});

$("#hanoi_input").click(function() {
    $('input[name="project"]').val("tower_of_hanoi");
    $('input[name="git_tag"]').val("v0.4");
});
$("#greetings_input").click(function() {
    $('input[name="project"]').val("greetings");
    $('input[name="git_tag"]').val("v0.3");
});
